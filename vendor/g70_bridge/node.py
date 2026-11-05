"""Single serial owner: RTCM input + checked NMEA output; never publishes odom or TF."""
import json,math,os,threading,time
from pathlib import Path
import yaml,serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix,NavSatStatus
from std_msgs.msg import String,Bool
from .protocol import gga,checked_nmea,MeasurementCache
from geometry_msgs.msg import TwistStamped
from .ntrip import NtripWorker
from .ubx import PvtDecoder

def load_config(path):
    path=Path(path).resolve();cfg=yaml.safe_load(path.read_text())
    n=cfg['ntrip'];secret=os.environ.get('NTRIP_PASSWORD')
    if secret is None and n.get('password_file'):
        p=path.parent/n['password_file']
        if p.exists():
            if p.stat().st_mode&0o077:raise ValueError('Password file must have mode 600')
            secret=p.read_text().rstrip('\r\n')
    if secret=='' and not n.get('allow_empty_password',False):secret=None
    return cfg,secret

def fix_message(data,header,sigma=None):
    msg=NavSatFix();msg.header=header
    valid=data is not None and data['valid']
    msg.status.status=(NavSatStatus.STATUS_GBAS_FIX if data['quality'] in (2,4,5) else NavSatStatus.STATUS_FIX) if valid else NavSatStatus.STATUS_NO_FIX
    # GNGGA does not identify which constellations contributed; do not invent service bits.
    msg.status.service=0
    msg.latitude=data['latitude'] if valid else math.nan
    msg.longitude=data['longitude'] if valid else math.nan
    msg.altitude=data['altitude_ellipsoid'] if valid else math.nan
    msg.position_covariance_type=NavSatFix.COVARIANCE_TYPE_UNKNOWN
    if valid and sigma:
        msg.position_covariance[0]=sigma['longitude_sigma_m']**2
        msg.position_covariance[4]=sigma['latitude_sigma_m']**2
        msg.position_covariance[8]=sigma['altitude_sigma_m']**2
        msg.position_covariance_type=NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return msg

class G70(Node):
    def __init__(self):
        super().__init__('g70_bridge')
        self.declare_parameter('config_file','')
        self.cfg,password=load_config(self.get_parameter('config_file').value)
        self.frame=self.cfg.get('frame_id','gnss_link')
        self.stale=float(self.cfg.get('fix_timeout',3.0))
        self.dev=None;self.lock=threading.Lock();self.buffer=bytearray()
        self.last=None;self.last_time=None;self.latest_gga=None;self.utc=None;self.last_measurement=None
        self.measurements=MeasurementCache();self.fix_stamp=None
        self.ubx=PvtDecoder();self.pvt=None;self.pvt_at=None
        self.ubx_mode=self.cfg.get("receiver_protocol","nmea")=="ubx_nav_pvt"
        self.velocity_pub=self.create_publisher(TwistStamped,'/gps/velocity',10)
        self.bad_nmea=0;self.serial_state='DISCONNECTED';self.serial_error='';self.next_open=0.
        self.fix_pub=self.create_publisher(NavSatFix,'/gps/fix',10)
        self.rtk_pub=self.create_publisher(String,'/gps/rtk_status',10)
        self.fixed_pub=self.create_publisher(Bool,'/gps/rtk_fixed',10)
        self.raw_pub=self.create_publisher(String,'/gps/nmea',10)
        self.worker=NtripWorker(self.cfg['ntrip'],password,self.fresh_gga,self.write_rtcm)
        self.worker.start()
        self.timer=self.create_timer(.02,self.read_serial)
        self.status_timer=self.create_timer(.5,self.publish_status)
        self.last_print=0.
        if not self.cfg['serial'].get('port'):
            self.get_logger().warn('Serial port not configured; use inspect.sh then set config/g70.yaml serial.port')
        if password is None:self.get_logger().warn('NTRIP password not configured; GNSS output remains available')
    def fresh_gga(self):
        with self.lock:
            if (self.last and self.last['valid'] and self.dev and self.last_time is not None
                    and time.monotonic()-self.last_time<self.stale
                    and time.monotonic()-self.last_measurement<self.stale):
                return self.latest_gga
        return None
    def disconnect(self,error):
        with self.lock:
            if self.dev:
                try:self.dev.close()
                except OSError:pass
            self.dev=None;self.latest_gga=None;self.last_time=None;self.last=None
            self.last_measurement=None;self.utc=None
        self.measurements=MeasurementCache();self.fix_stamp=None
        self.ubx=PvtDecoder();self.pvt=None;self.pvt_at=None
        self.buffer.clear();self.serial_state='DISCONNECTED';self.serial_error=type(error).__name__+': '+str(error)
        self.next_open=time.monotonic()+3
    def write_rtcm(self,frame):
        try:
            with self.lock:
                if self.dev is None:raise OSError('Serial disconnected')
                count=self.dev.write(frame)
                if count!=len(frame):raise OSError('Partial RTCM serial write')
        except (OSError,serial.SerialException) as e:
            self.disconnect(e);raise OSError('RTCM serial injection failed') from None
    def read_serial(self):
        try:
            if self.dev is None:
                if time.monotonic()<self.next_open:return
                self.next_open=time.monotonic()+3
                port=self.cfg['serial'].get('port')
                if not port:return
                # Explicit port only: never guess a board UART or claim any attached USB device.
                dev=serial.Serial(port,int(self.cfg['serial']['baud']),timeout=0,write_timeout=2,exclusive=True)
                with self.lock:self.dev=dev
                self.serial_state='OPEN';self.serial_error=''
            with self.lock:
                if self.dev is None:return
                data=self.dev.read(min(self.dev.in_waiting,16384))
            if self.ubx_mode:
                for sample in self.ubx.feed(data):self.publish_pvt(sample)
            self.buffer.extend(data)
            while b'\n' in self.buffer:
                line,_,remaining=self.buffer.partition(b'\n');self.buffer=bytearray(remaining)
                start=line.rfind(b'$')
                if start<0:continue
                line=bytes(line[start:]).strip()
                try:
                    checked_nmea(line);parsed=gga(line)
                except (ValueError,UnicodeError,OverflowError):
                    self.bad_nmea+=1;continue
                self.raw_pub.publish(String(data=line.decode('ascii')))
                if self.ubx_mode and parsed is None:continue
                try:
                    kind=self.measurements.feed(line,time.monotonic(),self.get_clock().now().to_msg())
                except ValueError:
                    self.bad_nmea+=1;continue
                if kind=='velocity':
                    v=self.measurements.velocity;msg=TwistStamped()
                    msg.header.stamp=self.measurements.velocity_stamp;msg.header.frame_id='gnss_enu'
                    msg.twist.linear.x=v['east_mps'];msg.twist.linear.y=v['north_mps']
                    self.velocity_pub.publish(msg)
                if parsed is None:
                    if kind=='sigma' and self.last and self.fix_stamp is not None:
                        self.publish_fix(self.last)
                    continue
                with self.lock:
                    now=time.monotonic()
                    if parsed['utc_time']!=self.utc:
                        self.last_measurement=now
                        if not self.ubx_mode:self.fix_stamp=self.get_clock().now().to_msg()
                    self.utc=parsed['utc_time'];self.last_time=now;self.last=parsed
                    self.latest_gga=line+b'\r\n'
                if not self.ubx_mode:self.publish_fix(parsed)
            if len(self.buffer)>8192:self.buffer.clear()
        except (OSError,serial.SerialException) as e:self.disconnect(e)
    def publish_pvt(self,sample):
        from std_msgs.msg import Header
        self.pvt=sample;self.pvt_at=time.monotonic();self.fix_stamp=self.get_clock().now().to_msg()
        sigma={'longitude_sigma_m':sample['hacc_m'],'latitude_sigma_m':sample['hacc_m'],'altitude_sigma_m':sample['vacc_m']}
        self.fix_pub.publish(fix_message(sample,Header(stamp=self.fix_stamp,frame_id=self.frame),sigma))
        velocity=TwistStamped();velocity.header.stamp=self.fix_stamp;velocity.header.frame_id='gnss_enu'
        velocity.twist.linear.x=sample['east_mps'];velocity.twist.linear.y=sample['north_mps']
        self.velocity_pub.publish(velocity)

    def publish_fix(self,data):
        from std_msgs.msg import Header
        sigma=self.measurements.position_sigma(data['utc_time'],time.monotonic(),self.stale) if data else None
        self.fix_pub.publish(fix_message(data,Header(stamp=self.fix_stamp if data and self.fix_stamp is not None else self.get_clock().now().to_msg(),frame_id=self.frame),sigma))
    def publish_status(self):
        now=time.monotonic()
        with self.lock:
            data=dict(self.last) if self.last else None
            age=now-self.last_time if self.last_time is not None else None
            measurement_age=now-self.last_measurement if self.last_measurement is not None else None
        fresh=(self.dev is not None and age is not None and age<self.stale
               and measurement_age is not None and measurement_age<self.stale)
        fixed=bool(fresh and data and data['valid'] and data['quality']==4)
        correction=self.worker.snapshot();rt=correction.pop('last_rtcm_monotonic')
        correction['rtcm_age_s']=None if rt is None else now-rt
        correction['corrections_fresh']=bool(rt is not None and now-rt<self.cfg['ntrip'].get('stream_timeout',15))
        result=dict(state=(data['state'] if fresh else 'STALE' if data else 'NO_DATA'),
            receiver_quality=data['quality'] if data else None,rtk_fixed=fixed,
            fix_stamp_s=self.fix_stamp.sec+self.fix_stamp.nanosec/1e9 if self.fix_stamp is not None else None,
            fix_age_s=age,measurement_age_s=measurement_age,serial_state=self.serial_state,
            serial_error=self.serial_error,bad_nmea=self.bad_nmea,ntrip=correction,
            velocity=self.measurements.velocity_status(now,self.stale),position_covariance_source='NMEA_GST' if data and self.measurements.position_sigma(data['utc_time'],now,self.stale) else 'UNKNOWN')
        if data:
            result.update(satellites=data['satellites'],hdop=data['hdop'],
                differential_age_s=data['differential_age_s'],utc_time=data['utc_time'])
        if self.ubx_mode:
            age=None if self.pvt_at is None else now-self.pvt_at
            fresh=bool(self.dev is not None and age is not None and 0<=age<self.stale)
            pvt=self.pvt
            quality=pvt['quality'] if pvt else 0
            stamp=None if self.fix_stamp is None else self.fix_stamp.sec+self.fix_stamp.nanosec/1e9
            result.update(state=pvt['state'] if fresh and pvt else 'STALE' if pvt else 'NO_DATA',
                receiver_quality=quality,rtk_fixed=bool(fresh and quality==4),fix_age_s=age,
                measurement_age_s=age,fix_stamp_s=stamp,
                measurement_id=pvt['tow_ms'] if pvt else None,epoch_source='UBX_iTOW_ms',
                position_covariance_source='UBX_NAV_PVT',
                velocity={'fresh':fresh,'age_s':age,'stamp_s':stamp,'source':'UBX_NAV_PVT',
                    'reference_frame':'ENU','measurement_id':pvt['tow_ms'] if pvt else None,
                    'accuracy_mps':pvt['speed_accuracy_mps'] if pvt else None,'accuracy_source':'UBX_NAV_PVT',
                    'covariance_known':False})
            fixed=result['rtk_fixed']
        self.fixed_pub.publish(Bool(data=fixed))
        self.rtk_pub.publish(String(data=json.dumps(result,allow_nan=False)))
        if not fresh:self.publish_fix(None)
        if now-self.last_print>=2:
            self.get_logger().info(json.dumps(result,allow_nan=False));self.last_print=now
    def close(self):
        self.worker.close();self.disconnect(OSError('stopped'))

def main():
    rclpy.init();node=None
    try:node=G70();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.close();node.destroy_node()
        rclpy.try_shutdown()
