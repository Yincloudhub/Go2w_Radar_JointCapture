from __future__ import annotations

import argparse
import json
import math
import hashlib
import hmac
import os
import socket
import shlex
import secrets
import statistics
import sys
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
VENDOR_DIR = REPO_ROOT.parent / "_vendor"
if VENDOR_DIR.exists():
    sys.path.insert(0, str(VENDOR_DIR))

try:
    import paramiko  # type: ignore
except ImportError:  # pragma: no cover
    paramiko = None

from radar_fleet_control import DEFAULT_CONFIG_PATH, load_config, normalize_config, resolve_secret, select_stations


def resolve_ssh_lookup(host: str, ssh_config_path: str | Path | None) -> dict[str, Any]:
    if ssh_config_path in (None, ""):
        return {}
    config_path = Path(ssh_config_path).expanduser()
    if not config_path.exists():
        return {}
    ssh_config = paramiko.SSHConfig()
    with config_path.open("r", encoding="utf-8") as config_file:
        ssh_config.parse(config_file)
    return ssh_config.lookup(host)


def resolve_identity_files(explicit_key_file: str, ssh_lookup: dict[str, Any]) -> list[str]:
    if explicit_key_file:
        return [str(Path(explicit_key_file).expanduser())]
    identity_files = ssh_lookup.get("identityfile") or []
    resolved: list[str] = []
    for identity_file in identity_files:
        resolved.append(str(Path(identity_file).expanduser()))
    return resolved


REMOTE_TIME_COMMAND = """python3 - <<'PY'
import json
import socket
import time
print(json.dumps({
  "hostname": socket.gethostname(),
  "wall_time_ns": time.time_ns(),
  "monotonic_ns": time.monotonic_ns()
}))
PY"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Measure rough controller-to-station wall-clock offsets over SSH."
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--station", action="append", default=[])
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--max-offset-ms", type=float, default=10.0)
    return parser.parse_args()


def create_station_client(station: dict[str, Any]) -> paramiko.SSHClient:
    if paramiko is None:
        raise RuntimeError("paramiko is required for fleet_clock_check.py")

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh_lookup = resolve_ssh_lookup(station["host"], station.get("ssh_config"))
    hostname = str(ssh_lookup.get("hostname") or station["host"])
    username = str(ssh_lookup.get("user") or station["user"])
    port = int(ssh_lookup.get("port") or station["port"])

    connect_kwargs: dict[str, Any] = {
        "hostname": hostname,
        "port": port,
        "username": username,
        "timeout": 15,
        "banner_timeout": 15,
        "auth_timeout": 15,
        "allow_agent": True,
        "look_for_keys": True,
    }
    key_files = resolve_identity_files(str(station.get("key_file", "")), ssh_lookup)
    if key_files:
        connect_kwargs["key_filename"] = key_files
    password = resolve_secret(station.get("password_env", ""))
    if password:
        connect_kwargs["password"] = password

    client.connect(**connect_kwargs)
    return client


def parse_json_blob(text: str) -> dict[str, Any]:
    payload = json.loads(text.strip())
    if not isinstance(payload, dict):
        raise RuntimeError("Remote time payload was not a JSON object.")
    return payload


REMOTE_PROBE = '''import sys,json,time,socket
print(json.dumps({'ready':True,'hostname':socket.gethostname()}),flush=True)
for line in sys.stdin:
 t1=time.time_ns();m1=time.monotonic_ns()
 seq=int(line.strip())
 m2=time.monotonic_ns();t2=time.time_ns()
 print(json.dumps({'sequence':seq,'receive_wall_ns':t1,'send_wall_ns':t2,
  'receive_monotonic_ns':m1,'send_monotonic_ns':m2}),flush=True)
'''

REMOTE_UDP_PROBE = '''import sys,json,time,socket,hmac,hashlib
key=bytes.fromhex(sys.argv[1]);s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.bind(('0.0.0.0',0));s.settimeout(1)
print(json.dumps({'ready':True,'hostname':socket.gethostname(),'port':s.getsockname()[1]}),flush=True)
deadline=time.monotonic()+15
while time.monotonic()<deadline:
 try: data,addr=s.recvfrom(4096)
 except socket.timeout: continue
 t1=time.time_ns();m1=time.monotonic_ns()
 try:
  envelope=json.loads(data);body=envelope['body']
  if not hmac.compare_digest(envelope['mac'],hmac.new(key,body.encode(),hashlib.sha256).hexdigest()):continue
  request=json.loads(body)
  if request.get('stop'):break
  reply={'sequence':request['sequence'],'receive_wall_ns':t1,'receive_monotonic_ns':m1}
  reply['send_monotonic_ns']=time.monotonic_ns();reply['send_wall_ns']=time.time_ns()
  body=json.dumps(reply,separators=(',',':'))
  s.sendto(json.dumps({'body':body,'mac':hmac.new(key,body.encode(),hashlib.sha256).hexdigest()}).encode(),addr)
 except (ValueError,KeyError,TypeError):continue
s.close()
'''


def four_timestamp_sample(t0, t3, local_elapsed_ns, payload):
    t1, t2 = int(payload['receive_wall_ns']), int(payload['send_wall_ns'])
    remote_elapsed = int(payload['send_monotonic_ns']) - int(payload['receive_monotonic_ns'])
    if abs((t3-t0)-local_elapsed_ns) > 500_000 or abs((t2-t1)-remote_elapsed) > 500_000:
        raise RuntimeError('Clock stepped during measurement; retry only after clock discipline settles.')
    delay = (t3-t0)-(t2-t1)
    if delay < 0 or t3 < t0 or t2 < t1:
        raise RuntimeError('Invalid four-timestamp ordering.')
    offset = ((t1-t0)+(t2-t3))/2
    uncertainty = delay/2 + 100_000  # include timestamp-read/scheduling margin
    return {'controller_send_wall_ns':t0,'controller_recv_wall_ns':t3,
            'remote_receive_wall_ns':t1,'remote_send_wall_ns':t2,
            'rtt_ms':delay/1e6,'offset_ms':offset/1e6,
            'uncertainty_ms':uncertainty/1e6,
            'max_possible_abs_offset_ms':(abs(offset)+uncertainty)/1e6}


class RemoteClockProbe:
    def __init__(self, client):
        transport = client.get_transport()
        self.key=secrets.token_bytes(32)
        self.stdin,self.stdout,self.stderr = client.exec_command(
            'python3 -u -c '+shlex.quote(REMOTE_UDP_PROBE)+' '+self.key.hex(),timeout=8)
        ready = parse_json_blob(self.stdout.readline())
        if ready.get('ready') is not True:
            raise RuntimeError('Persistent clock probe failed to start.')
        self.hostname=ready['hostname'];self.sequence=0
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.socket.settimeout(2)
        self.socket.connect((transport.sock.getpeername()[0],int(ready['port'])))

    def envelope(self,request):
        body=json.dumps(request,separators=(',',':'))
        return json.dumps({'body':body,'mac':hmac.new(self.key,body.encode(),hashlib.sha256).hexdigest()}).encode()

    def sample(self):
        self.sequence+=1
        request=self.envelope({'sequence':self.sequence})
        m0=time.monotonic_ns();t0=time.time_ns()
        self.socket.send(request)
        line=self.socket.recv(4096)
        t3=time.time_ns();m3=time.monotonic_ns()
        envelope=json.loads(line)
        if not hmac.compare_digest(envelope['mac'],hmac.new(self.key,envelope['body'].encode(),hashlib.sha256).hexdigest()):
            raise RuntimeError('Unauthenticated clock response rejected.')
        p=parse_json_blob(envelope['body'])
        if p.get('sequence')!=self.sequence:raise RuntimeError('Clock probe sequence mismatch.')
        return dict(four_timestamp_sample(t0,t3,m3-m0,p),remote_hostname=self.hostname)

    def close(self):
        try:self.socket.send(self.envelope({'stop':True}))
        finally:self.socket.close()
        self.stdout.channel.close()


def summarize_samples(station_id: str, host: str, samples: list[dict[str, Any]], max_offset_ms: float) -> dict[str, Any]:
    rtts = [float(sample["rtt_ms"]) for sample in samples]
    offsets = [float(sample["offset_ms"]) for sample in samples]
    median_rtt_ms = statistics.median(rtts) if rtts else None
    median_offset_ms = statistics.median(offsets) if offsets else None
    mean_offset_ms = statistics.mean(offsets) if offsets else None
    offset_stddev_ms = statistics.pstdev(offsets) if len(offsets) > 1 else 0.0 if offsets else None
    rtt_stddev_ms = statistics.pstdev(rtts) if len(rtts) > 1 else 0.0 if rtts else None
    max_abs_offset_ms = max((abs(item) for item in offsets), default=None)
    min_offset_ms = min(offsets, default=None)
    max_offset_observed_ms = max(offsets, default=None)
    min_rtt_ms = min(rtts, default=None)
    best_sample = min(samples, key=lambda item: float(item["rtt_ms"])) if samples else None
    best_effort_offset_ms = float(best_sample["offset_ms"]) if best_sample is not None else None
    bound = float(best_sample.get('max_possible_abs_offset_ms', math.inf)) if best_sample else math.inf
    status = (
        "ok"
        if best_effort_offset_ms is not None and math.isfinite(bound) and bound <= max_offset_ms
        else "offset_exceeds_threshold"
    )
    return {
        "station_id": station_id,
        "host": host,
        "status": status,
        "sample_count": len(samples),
        "median_rtt_ms": median_rtt_ms,
        "median_offset_ms": median_offset_ms,
        "mean_offset_ms": mean_offset_ms,
        "offset_stddev_ms": offset_stddev_ms,
        "rtt_stddev_ms": rtt_stddev_ms,
        "min_rtt_ms": min_rtt_ms,
        "min_offset_ms": min_offset_ms,
        "max_offset_observed_ms": max_offset_observed_ms,
        "best_effort_offset_ms": best_effort_offset_ms,
        "measurement_method": "authenticated_udp_four_timestamp",
        "uncertainty_ms": best_sample.get('uncertainty_ms') if best_sample else None,
        "max_possible_abs_offset_ms": bound if math.isfinite(bound) else None,
        "max_abs_offset_ms": max_abs_offset_ms,
        "threshold_ms": max_offset_ms,
        "samples": samples,
    }


def main() -> int:
    args = parse_args()
    config = normalize_config(load_config(args.config), args.config)
    stations = select_stations(config, args.station)

    station_reports: list[dict[str, Any]] = []
    overall_status = "ok"
    for station in stations:
        client: paramiko.SSHClient | None = None
        try:
            client = create_station_client(station)
            samples: list[dict[str, Any]] = []
            probe = RemoteClockProbe(client)
            try:
                for _ in range(2):probe.sample()  # connection warm-up, not counted
                for _ in range(max(100, args.samples)):
                    samples.append(probe.sample())
                    # Allow early completion only after a useful observation set
                    # and with the same bounded-evidence policy used for admission.
                    if len(samples)>=25:
                        best=min(samples,key=lambda sample:sample['rtt_ms'])
                        if best['max_possible_abs_offset_ms'] <= args.max_offset_ms:
                            break
                    time.sleep(0.03)
            finally:
                probe.close()
            report = summarize_samples(station["station_id"], station["host"], samples, args.max_offset_ms)
        except Exception as exc:
            overall_status = "error"
            report = {
                "station_id": station["station_id"],
                "host": station["host"],
                "status": "error",
                "message": f"SSH/time query failed for {station['station_id']} ({station['host']}): {exc}",
                "sample_count": 0,
                "samples": [],
            }
        finally:
            if client is not None:
                client.close()

        if report["status"] != "ok":
            overall_status = "error" if report["status"] == "error" else "warning"
        station_reports.append(report)

    payload = {
        "status": overall_status,
        "checked_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
        "controller": {
            "hostname": socket.gethostname(),
            "user": os.environ.get("USER") or os.environ.get("USERNAME") or "",
        },
        "max_offset_ms": args.max_offset_ms,
        "stations": station_reports,
        "notes": [
            "SSH-authenticated temporary UDP probe, HMAC-authenticated four-timestamp exchange; no permanent open port.",
            "Admission uses abs(offset) + network uncertainty + 0.1ms timestamp margin, not offset alone.",
            "Chrony/PTP remains necessary for clock discipline; this does not calibrate camera exposure or sensor latency.",
        ],
    }
    print(json.dumps(payload, indent=2))
    return 0 if overall_status == "ok" else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(json.dumps({"status": "error", "message": str(exc)}), file=sys.stderr)
        raise SystemExit(2)
