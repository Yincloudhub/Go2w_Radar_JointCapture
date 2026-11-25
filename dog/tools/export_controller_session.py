"""Forced-command exporter: read only, limited to completed fleet-session artifacts."""
import argparse
import hashlib, io, json, os, shlex, sys, tarfile
from pathlib import Path
DEFAULT_CONFIG = Path(__file__).resolve().with_name('radar_fleet_config.json')
ROOT_FILES={'fleet_manifest.json','fleet_config_snapshot.json'}
REPORT_FILES={'clock_before_capture.json','clock_after_capture.json','capture_overview.md','controller_imu_capture.json','controller_video_capture.json','operator_request.json','station_storage_summary.json'}


def find_session(name, config_paths):
    if not name or Path(name).name != name or name in {'.', '..'}:
        raise ValueError('Invalid session name')
    roots = set()
    for config_path in config_paths:
        config_path = config_path.expanduser().resolve()
        config = json.loads(config_path.read_text(encoding='utf-8'))
        output_root = Path(config.get('output_root', 'data/sessions')).expanduser()
        roots.add((config_path.parent.parent / output_root).resolve())

    matches = []
    for root in sorted(roots):
        directory = root / name
        manifest = directory / 'fleet_manifest.json'
        if directory.is_symlink() or manifest.is_symlink():
            raise ValueError('Symbolic links are not permitted in the session lookup')
        if directory.is_dir() and manifest.is_file():
            matches.append(directory)
    if len(matches) != 1:
        if not matches:
            raise FileNotFoundError('Session not found in the configured output roots')
        raise ValueError('Session name is ambiguous across configured output roots')
    return matches[0]


def allowed(name):
    parts=Path(name).parts
    return (len(parts)==1 and parts[0] in ROOT_FILES) or (len(parts)==2 and parts[0] in {'controller_imu','controller_video'}) or (len(parts)==2 and parts[0]=='reports' and parts[1] in REPORT_FILES)
def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()
def export(name, config_paths):
    directory = find_session(name, config_paths)
    manifest=json.loads((directory/'fleet_manifest.json').read_text())
    if not manifest.get('capture_id'):raise ValueError('Missing capture_id')
    paths=[]
    for p in sorted(directory.rglob('*')):
        rel=p.relative_to(directory).as_posix()
        if not allowed(rel):continue
        if p.is_symlink() or not p.is_file() or directory.resolve() not in p.resolve().parents:raise ValueError('Unsafe file')
        paths.append((rel,p))
    files={rel:dict(bytes=p.stat().st_size,sha256=digest(p)) for rel,p in paths}
    transfer=dict(schema_version=1,session_name=name,capture_id=manifest['capture_id'],files=files)
    raw=json.dumps(transfer,indent=2).encode()
    with tarfile.open(fileobj=sys.stdout.buffer,mode='w|') as tar:
        info=tarfile.TarInfo('controller_bundle_manifest.json');info.size=len(raw);tar.addfile(info,io.BytesIO(raw))
        for rel,p in paths:tar.add(p,arcname=rel,recursive=False)
if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', action='append', type=Path,
                        help='Allow exports from this controller configuration output root.')
    server_args = parser.parse_args()
    try:
        args=shlex.split(os.environ.get('SSH_ORIGINAL_COMMAND',''))
        if len(args)!=2 or args[0]!='export-session':raise ValueError('Only export-session NAME is permitted')
        export(args[1], server_args.config or [DEFAULT_CONFIG])
    except Exception as exc:
        print(str(exc),file=sys.stderr);raise SystemExit(2)
