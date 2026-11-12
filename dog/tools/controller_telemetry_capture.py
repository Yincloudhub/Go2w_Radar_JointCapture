from __future__ import annotations
import argparse, csv, getpass, json, math, signal, socket, time
from pathlib import Path
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry
    from unitree_go.msg import SportModeState
except ImportError as exc:
    rclpy = None
    Node = object
    IMPORT_ERROR = str(exc)

STOP = False
COMMON = ['sample_index','host_wall_time_ns','host_monotonic_ns','message_stamp_sec','message_stamp_nanosec','frame_id']
def xyz(v): return [float(v.x),float(v.y),float(v.z)]
def quat(v): return xyz(v)+[float(v.w)]
def valid_quat(q): return all(math.isfinite(x) for x in q) and abs(sum(x*x for x in q)-1)<.05

def imu_row(m):
    q=quat(m.orientation)
    keys=['orientation_x','orientation_y','orientation_z','orientation_w','angular_velocity_x','angular_velocity_y','angular_velocity_z','linear_acceleration_x','linear_acceleration_y','linear_acceleration_z']
    return dict(zip(keys,q+xyz(m.angular_velocity)+xyz(m.linear_acceleration)),orientation_valid=int(valid_quat(q) and m.orientation_covariance[0]!=-1),orientation_covariance=json.dumps(list(m.orientation_covariance)),angular_velocity_covariance=json.dumps(list(m.angular_velocity_covariance)),linear_acceleration_covariance=json.dumps(list(m.linear_acceleration_covariance)))

def odom_row(m):
    p,t=m.pose.pose,m.twist.twist
    q=quat(p.orientation)
    keys=['position_x','position_y','position_z','orientation_x','orientation_y','orientation_z','orientation_w','linear_velocity_x','linear_velocity_y','linear_velocity_z','angular_velocity_x','angular_velocity_y','angular_velocity_z']
    return dict(zip(keys,xyz(p.position)+q+xyz(t.linear)+xyz(t.angular)),child_frame_id=m.child_frame_id,orientation_valid=int(valid_quat(q)),speed_mps=math.sqrt(sum(v*v for v in xyz(t.linear))),pose_covariance=json.dumps(list(m.pose.covariance)),twist_covariance=json.dumps(list(m.twist.covariance)))

def sport_row(m):
    keys=['position_x','position_y','position_z','velocity_x','velocity_y','velocity_z','imu_quaternion_w','imu_quaternion_x','imu_quaternion_y','imu_quaternion_z','imu_gyro_x','imu_gyro_y','imu_gyro_z','imu_accel_x','imu_accel_y','imu_accel_z']
    return dict(zip(keys,list(m.position)+list(m.velocity)+list(m.imu_state.quaternion)+list(m.imu_state.gyroscope)+list(m.imu_state.accelerometer)),yaw_speed=float(m.yaw_speed),mode=int(m.mode),gait_type=int(m.gait_type),error_code=int(m.error_code),orientation_valid=int(valid_quat(list(m.imu_state.quaternion))),speed_mps=math.sqrt(sum(float(v)**2 for v in m.velocity)))

class CsvStream:
    def __init__(self,topic,filename,converter,directory=None):
        self.topic,self.filename,self.converter=topic,filename,converter
        self.count=0; self.first=self.last=self.sample=None
        self.invalid_orientation=self.invalid_stamp=self.nonfinite=self.reversed_stamp=0
        self.max_gap=self.max_clock_delta=0; self.max_speed=None
        self.file=(directory/filename).open('x',newline='',encoding='utf-8') if directory else None
        self.writer=None; self.flushed=0
    def callback(self,m):
        wall,mono=time.time_ns(),time.monotonic_ns()
        header=getattr(m,'header',None); stamp=header.stamp if header is not None else m.stamp
        source=int(stamp.sec)*1000000000+int(stamp.nanosec)
        values=self.converter(m)
        row=dict(zip(COMMON,[self.count+1,wall,mono,int(stamp.sec),int(stamp.nanosec),header.frame_id if header else '']))
        row.update(values); row['message_stamp_valid']=int(source>0)
        row['source_clock_delta_ns']=wall-source if source>0 else ''
        row['numeric_values_finite']=int(all(math.isfinite(v) for v in values.values() if isinstance(v,float)))
        self.invalid_stamp+=int(source<=0); self.invalid_orientation+=int(not values.get('orientation_valid',True))
        self.nonfinite+=int(not row['numeric_values_finite'])
        if self.last:
            self.max_gap=max(self.max_gap,mono-self.last[1])
            self.reversed_stamp+=int(source>0 and self.last[2]>0 and source<self.last[2])
        if source>0:self.max_clock_delta=max(self.max_clock_delta,abs(wall-source))
        speed=values.get('speed_mps')
        if speed is not None and math.isfinite(speed):self.max_speed=max(self.max_speed or 0,speed)
        self.count+=1; self.first=self.first or (wall,mono,source); self.last=(wall,mono,source); self.sample=row
        if self.file:
            if self.writer is None:
                self.writer=csv.DictWriter(self.file,fieldnames=list(row)); self.writer.writeheader()
            self.writer.writerow(row)
            if self.count==1 or mono-self.flushed>=100000000:
                self.file.flush(); self.flushed=mono
    def summary(self):
        duration=(self.last[1]-self.first[1])/1e9 if self.count>1 else 0
        return dict(topic=self.topic,csv_file=self.filename,sample_count=self.count,
            first_sample_wall_time_ns=self.first[0] if self.first else None,last_sample_wall_time_ns=self.last[0] if self.last else None,
            first_sample_monotonic_ns=self.first[1] if self.first else None,last_sample_monotonic_ns=self.last[1] if self.last else None,
            first_message_stamp_ns=self.first[2] if self.first else None,last_message_stamp_ns=self.last[2] if self.last else None,
            received_duration_seconds=duration,observed_rate_hz=(self.count-1)/duration if duration else None,
            max_receive_gap_seconds=self.max_gap/1e9,invalid_orientation_count=self.invalid_orientation,
            invalid_message_stamp_count=self.invalid_stamp,out_of_order_message_stamp_count=self.reversed_stamp,
            nonfinite_sample_count=self.nonfinite,max_abs_source_clock_delta_seconds=self.max_clock_delta/1e9,max_speed_mps=self.max_speed)
    def close(self):
        if self.file:self.file.close()

class TelemetryNode(Node):
    def __init__(self,topic,odom_topic='',sport_topic='',lidar_imu_topic='',directory=None):
        super().__init__('controller_telemetry_recorder' if directory else 'controller_telemetry_probe')
        specs=[('imu',topic,'imu_samples.csv',Imu,imu_row)]
        if odom_topic:specs.append(('odom',odom_topic,'odom_samples.csv',Odometry,odom_row))
        if sport_topic:specs.append(('sport',sport_topic,'sport_samples.csv',SportModeState,sport_row))
        if lidar_imu_topic:specs.append(('lidar_imu',lidar_imu_topic,'lidar_imu_samples.csv',Imu,imu_row))
        self.streams={}; self.subscriptions_kept=[]
        for key,topic,filename,typ,converter in specs:
            stream=CsvStream(topic,filename,converter,directory); self.streams[key]=stream
            self.subscriptions_kept.append(self.create_subscription(typ,topic,stream.callback,qos_profile_sensor_data))
    def manifest(self,controller_id,capture_id,name,directory):
        streams={k:s.summary() for k,s in self.streams.items()}
        for key,stream in streams.items():
            stream['publisher_count']=self.count_publishers(stream['topic'])
            if self.streams[key].last:
                stream['last_receive_age_seconds']=max(0.,(time.monotonic_ns()-self.streams[key].last[1])/1e9)
        missing=[k for k,s in streams.items() if not s['sample_count']]
        missing_topics=[streams[k]['topic'] for k in missing]
        bad=[k for k,s in streams.items() if s['nonfinite_sample_count']]
        warnings=[]
        for topic in missing_topics:
            warnings.append(f'{topic}: no messages received. Check whether its publisher has been started manually before capture.')
        for k,s in streams.items():
            if s['invalid_orientation_count']:warnings.append(k+': invalid quaternion; do not use as orientation.')
            if s['invalid_message_stamp_count'] or s['max_abs_source_clock_delta_seconds']>1:
                warnings.append(k+': invalid source stamp or source clock differs from host; preserve stamps and use clock audit/receive times for software alignment.')
            if s['max_receive_gap_seconds']>.5:warnings.append(k+': receive gap exceeds 0.5 seconds.')
        result=dict(streams['imu']); first=self.streams['imu'].first; last=self.streams['imu'].last
        result.update(schema_version=2,status='capture_failed' if missing or bad else 'ok',controller_id=controller_id,capture_id=capture_id,requested_name=name,
            created_at_local=time.strftime('%Y-%m-%d %H:%M:%S'),host=dict(hostname=socket.gethostname(),user=getpass.getuser()),output_dir=str(directory),streams=streams,warnings=warnings,
            first_message_stamp_sec=first[2]//1000000000 if first else None,first_message_stamp_nanosec=first[2]%1000000000 if first else None,
            last_message_stamp_sec=last[2]//1000000000 if last else None,last_message_stamp_nanosec=last[2]%1000000000 if last else None,
            error=(f'No messages received from: {", ".join(missing_topics)}. '
                   f'Check whether the topic publisher needs to be started manually before capture. '
                   f'Nonfinite streams: {bad}' if missing or bad else None),
            timestamp_semantics='host_monotonic_ns is controller-local only; host_wall_time_ns is receive time; message stamps retain source clocks. Software synchronization only.',
            units=dict(linear_acceleration='m/s^2, gravity may be included',angular_velocity='rad/s',velocity='m/s',position='m'),
            frame_semantics='Preserve IMU frame_id and odometry frame_id/child_frame_id. Sport frame is not declared in its message; no conversion applied. Unitree quaternion component names retain wxyz order. Zero pose/velocity while lying down does not establish valid localization.')
        return result

def stop(_signum,_frame):
    global STOP
    STOP=True

def main():
    global STOP
    p=argparse.ArgumentParser(description='Read-only joint capture of controller IMU, odometry and sport state.')
    sub=p.add_subparsers(dest='command',required=True)
    for command in ('preflight','record'):
        a=sub.add_parser(command); a.add_argument('--topic',default='/dog_imu_raw')
        for key in ('odom','sport','lidar-imu'):a.add_argument('--'+key+'-topic',default='')
        if command=='preflight':a.add_argument('--timeout-seconds',type=float,default=3)
        else:
            a.add_argument('--output-dir',type=Path,required=True); a.add_argument('--controller-id',default='dog_nx')
            a.add_argument('--capture-id',default=''); a.add_argument('--name',default=''); a.add_argument('--duration-seconds',type=float,default=0)
    args=p.parse_args()
    if rclpy is None:
        print(json.dumps(dict(status='error',error=IMPORT_ERROR))); return 2
    directory=args.output_dir.expanduser() if args.command=='record' else None
    if directory:
        directory.mkdir(parents=True,exist_ok=True)
        for filename in ('imu_manifest.json','imu_samples.csv','odom_samples.csv','sport_samples.csv','lidar_imu_samples.csv'):
            if (directory/filename).exists():raise RuntimeError('Refusing to overwrite existing telemetry: '+str(directory/filename))
    rclpy.init(args=None)
    node=TelemetryNode(args.topic,args.odom_topic,args.sport_topic,args.lidar_imu_topic,directory)
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    started=time.monotonic(); deadline=started+args.timeout_seconds if args.command=='preflight' else started+args.duration_seconds if args.duration_seconds>0 else math.inf
    try:
        while rclpy.ok() and not STOP and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            # Observe the entire window. One message cannot establish continuity.
    finally:
        result=node.manifest(getattr(args,'controller_id','dog_nx'),getattr(args,'capture_id',''),getattr(args,'name',''),directory)
        result['observation_seconds']=time.monotonic()-started
        for stream in node.streams.values():stream.close()
        node.destroy_node(); rclpy.shutdown()
    if directory:
        path=directory/'imu_manifest.json'; temp=path.with_suffix('.json.tmp')
        temp.write_text(json.dumps(result,indent=2),encoding='utf-8'); temp.replace(path)
        print(json.dumps(dict(status=result['status'],manifest=result),indent=2))
    else:
        if result['status']!='ok':result['status']='preflight_failed'
        result['sample']=node.streams['imu'].sample
        print(json.dumps(result,indent=2))
    return 0 if result['status']=='ok' else 2

if __name__=='__main__':raise SystemExit(main())
