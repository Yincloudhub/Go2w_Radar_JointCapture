"""Forced-command exporter: read only, limited to completed fleet-session artifacts."""
import hashlib, io, json, os, shlex, sys, tarfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]/'data'/'sessions'
ROOT_FILES={'fleet_manifest.json','fleet_config_snapshot.json'}
REPORT_FILES={'clock_before_capture.json','clock_after_capture.json','capture_overview.md','controller_imu_capture.json','controller_video_capture.json','operator_request.json','station_storage_summary.json'}
def allowed(name):
    parts=Path(name).parts
    return (len(parts)==1 and parts[0] in ROOT_FILES) or (len(parts)==2 and parts[0] in {'controller_imu','controller_video'}) or (len(parts)==2 and parts[0]=='reports' and parts[1] in REPORT_FILES)
def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()
def export(name):
    if not name or Path(name).name!=name or name in {'.','..'}:raise ValueError('Invalid session name')
    directory=ROOT/name
    if directory.is_symlink() or directory.resolve().parent!=ROOT.resolve():raise ValueError('Outside session root')
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
    try:
        args=shlex.split(os.environ.get('SSH_ORIGINAL_COMMAND',''))
        if len(args)!=2 or args[0]!='export-session':raise ValueError('Only export-session NAME is permitted')
        export(args[1])
    except Exception as exc:
        print(str(exc),file=sys.stderr);raise SystemExit(2)
