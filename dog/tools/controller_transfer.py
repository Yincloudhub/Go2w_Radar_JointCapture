"""Ask each radar NX to pull its matching controller artifacts after recording stops."""
import json, shlex, socket, time
from argparse import Namespace
from radar_station_remote import create_ssh_client

def transfer_controller_session(config,stations,records,session_dir,capture_id,secret_resolver):
    settings=config.get('controller_data_transfer') or {}
    if not settings.get('enabled'):return {'status':'disabled','stations':[]}
    results=[]
    for station in stations:
        record=next((r for r in records if r['station_id']==station['station_id']),{})
        capture=record.get('capture') or {}; response=capture.get('response') or {}
        remote_dir=(response.get('report') or {}).get('session_dir') or response.get('session_dir')
        if not remote_dir:
            results.append(dict(station_id=station['station_id'],status='error',error='No radar session path returned'));continue
        args=Namespace(host=station['host'],user=station['user'],port=station['port'],key_file=station['key_file'],ssh_config=station['ssh_config'],password=secret_resolver(station['password_env']))
        controller_host=settings.get('controller_host','auto')
        if controller_host=='auto':
            try:
                with socket.create_connection((station['host'],station['port']),timeout=5) as route:
                    controller_host=route.getsockname()[0]
            except OSError as exc:
                results.append(dict(station_id=station['station_id'],status='error',error='No route to radar: '+str(exc)));continue
        parts=['python3',station['remote_root']+'/tools/pull_controller_session.py','--controller-host',controller_host,'--source-session',session_dir.name,'--capture-id',capture_id,'--station-session',remote_dir]
        command=' '.join(shlex.quote(str(p)) for p in parts)
        result=None
        for attempt in range(2):
            client=None
            try:
                client=create_ssh_client(args)
                _,out,err=client.exec_command(command,timeout=330)
                stdout=out.read().decode();stderr=err.read().decode();code=out.channel.recv_exit_status()
                result=json.loads(stdout);result['returncode']=code
                if code or result.get('status')!='ok':raise RuntimeError(result.get('error') or stderr[-1000:] or 'Pull failed')
                break
            except Exception as exc:
                result=dict(status='error',error=str(exc),attempt=attempt+1)
                if attempt==0:time.sleep(1)
            finally:
                if client:client.close()
        result['station_id']=station['station_id'];results.append(result)
    return dict(status='ok' if results and all(r['status']=='ok' for r in results) else 'error',direction='radar_pulls_controller',stations=results)

