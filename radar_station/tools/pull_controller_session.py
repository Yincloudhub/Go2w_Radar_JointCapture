"""Radar NX pulls a matching controller session using a dedicated, restricted SSH key."""
import argparse, hashlib, json, os, shutil, subprocess, tarfile, tempfile, time
from pathlib import Path, PurePosixPath
STATION_CONFIG = Path(os.environ.get(
    'RADAR_STATION_CONFIG', str(Path(__file__).resolve().parents[1] / 'radar_station_config.json')
)).expanduser()


def capture_root(config_path):
    config_path = config_path.resolve()
    config = json.loads(config_path.read_text(encoding='utf-8'))
    output = Path(config['output_root']).expanduser()
    if output.is_absolute():
        return output.resolve()
    config_dir = config_path.parent
    workspace = config_dir.parent if config_dir.name == 'station' else config_dir
    candidate = workspace / output
    return (candidate if candidate.exists() else config_dir / output).resolve()


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()
def atomic_json(path,value):
    temp=path.with_suffix(path.suffix+'.tmp');temp.write_text(json.dumps(value,indent=2),encoding='utf-8');temp.replace(path)
def safe_member(name):
    p=PurePosixPath(name)
    if p.is_absolute() or '..' in p.parts or '\\' in name or not p.parts:raise ValueError('Unsafe archive path')
    return p

def unpack(archive,stage,capture_id,session_name):
    with tarfile.open(archive,'r:') as tar:
        members=tar.getmembers(); names=[m.name for m in members]
        if len(names)!=len(set(names)) or len(names)>1000:raise ValueError('Duplicate/excessive archive members')
        for m in members:
            safe_member(m.name)
            if not m.isfile():raise ValueError('Only regular files may be transferred')
        info=tar.getmember('controller_bundle_manifest.json')
        if info.size>1024*1024:raise ValueError('Oversized bundle manifest')
        manifest=json.load(tar.extractfile(info))
        if manifest['capture_id']!=capture_id or manifest['session_name']!=session_name:raise ValueError('Controller capture/session mismatch')
        expected=manifest['files']
        if set(names)!=set(expected)|{'controller_bundle_manifest.json'}:raise ValueError('Unexpected archive contents')
        for name,spec in expected.items():
            member=tar.getmember(name)
            if member.size!=spec['bytes']:raise ValueError('Size mismatch: '+name)
            destination=stage/str(safe_member(name));destination.parent.mkdir(parents=True,exist_ok=True)
            with tar.extractfile(member) as source,destination.open('xb') as target:shutil.copyfileobj(source,target,1024*1024)
            if digest(destination)!=spec['sha256']:raise ValueError('Checksum mismatch: '+name)
        atomic_json(stage/'controller_bundle_manifest.json',manifest)
        return manifest

def alignment(directory,station):
    def load(p):return json.loads(p.read_text()) if p.exists() else {}
    telemetry=load(directory/'controller_imu/imu_manifest.json');video=load(directory/'controller_video/controller_video_manifest.json')
    events=(station.get('radar') or {}).get('events',[])
    event_map={e.get('event_name') or e.get('name'):e for e in events}
    start=(event_map.get('sensor_start_sent') or {}).get('host_wall_time_ns')
    end=(event_map.get('sensor_stop_detected') or {}).get('host_wall_time_ns')
    rows={key:dict(first_wall_ns=s.get('first_sample_wall_time_ns'),last_wall_ns=s.get('last_sample_wall_time_ns'),samples=s.get('sample_count')) for key,s in telemetry.get('streams',{}).items()}
    rows['controller_video']=dict(first_wall_ns=video.get('first_frame_wall_time_ns'),last_wall_ns=video.get('last_frame_wall_time_ns'),samples=video.get('saved_frames'))
    for s in rows.values():
        a,b=s['first_wall_ns'],s['last_wall_ns']
        s['nominally_covers_radar_window']=bool(start and end and a and b and a<=start and b>=end)
        s['preroll_seconds']=(start-a)/1e9 if start and a else None
        s['postroll_seconds']=(b-end)/1e9 if end and b else None
    return dict(software_sync_only=True,radar_start_wall_ns=start,radar_stop_wall_ns=end,streams=rows,
        clock_evidence=['reports/clock_before_capture.json','reports/clock_after_capture.json'],
        warning='Cross-host coverage is nominal host-receive-time coverage, subject to measured clock offset, Wi-Fi/ROS latency and sensor pipeline delay; source timestamps are not rewritten.')

def pull(args):
    dest=Path(args.station_session).resolve()
    if capture_root(STATION_CONFIG) not in dest.parents:raise ValueError('Station session outside capture root')
    station=json.loads((dest/'session_manifest.json').read_text())
    if station.get('capture_id')!=args.capture_id:raise ValueError('Radar capture_id mismatch')
    final=dest/'controller_data'
    if final.exists():
        prior=json.loads((final/'controller_bundle_manifest.json').read_text())
        if prior['capture_id']!=args.capture_id or prior['session_name']!=args.source_session:raise ValueError('Existing destination belongs to a different session')
        if any(digest(final/name)!=spec['sha256'] for name,spec in prior['files'].items()):raise ValueError('Existing copy checksum mismatch')
        return dict(status='ok',already_present=True,output_dir=str(final),file_count=len(prior['files']))
    archive=dest/'.controller_bundle.partial.tar'
    key=Path.home()/'.ssh/fleet_controller_pull_ed25519'
    known=Path.home()/'.ssh/fleet_controller_known_hosts'
    command=['ssh','-T','-i',str(key),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','UserKnownHostsFile='+str(known),'-o','ConnectTimeout=10','unitree@'+args.controller_host,'export-session',args.source_session]
    with archive.open('wb') as output:
        completed=subprocess.run(command,stdout=output,stderr=subprocess.PIPE,timeout=300)
    if completed.returncode:raise RuntimeError('Controller pull failed: '+completed.stderr.decode(errors='replace')[-1500:])
    if shutil.disk_usage(dest).free<archive.stat().st_size+50*1024*1024:raise RuntimeError('Insufficient disk space for controller data')
    stage=Path(tempfile.mkdtemp(prefix='.controller_data.partial_',dir=dest))
    manifest=unpack(archive,stage,args.capture_id,args.source_session)
    atomic_json(stage/'joint_alignment.json',alignment(stage,station))
    stage.replace(final);archive.unlink()
    report=dict(status='ok',capture_id=args.capture_id,source_session=args.source_session,controller_host=args.controller_host,output_dir=str(final),file_count=len(manifest['files']),bytes=sum(s['bytes'] for s in manifest['files'].values()),completed_wall_ns=time.time_ns())
    atomic_json(dest/'controller_transfer.json',report)
    station['controller_data']=dict(directory='controller_data',transfer_report='controller_transfer.json',capture_id=args.capture_id)
    atomic_json(dest/'session_manifest.json',station)
    return report
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--controller-host',required=True);p.add_argument('--source-session',required=True);p.add_argument('--capture-id',required=True);p.add_argument('--station-session',required=True);a=p.parse_args()
    try:result=pull(a)
    except Exception as exc:result=dict(status='error',error=str(exc))
    print(json.dumps(result,indent=2));raise SystemExit(0 if result['status']=='ok' else 2)
