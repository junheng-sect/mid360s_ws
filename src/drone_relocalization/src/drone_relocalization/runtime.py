"""Read-only ROS interface. Dynamic drone_map->drone_odom is emitted only with fresh valid odometry."""
from collections import deque
import json
import math
import os
import threading
import time
import numpy as np
from .controller import StartupController
from .frames import ODOM,ODOM_EDGE,MOUNT_EDGE,init_edge
from .geometry import quat_R,R_quat,mapped_pose,unpack,yaw
from .matching import relocalize


def cloud2_xyz(m):
    fields={f.name:f for f in m.fields}
    if not all(n in fields and fields[n].datatype==7 and fields[n].count==1 for n in ('x','y','z')):
        raise ValueError('PointCloud2 requires scalar FLOAT32 xyz')
    if m.height<1 or m.width<1:return np.zeros((0,3))
    if m.row_step<m.width*m.point_step or len(m.data)<m.row_step*m.height:raise ValueError('truncated PointCloud2')
    endian='>' if m.is_bigendian else '<'
    dt=np.dtype(dict(names=['x','y','z'],formats=[endian+'f4']*3,
                     offsets=[fields[n].offset for n in ('x','y','z')],itemsize=m.point_step))
    a=np.ndarray((m.height,m.width),dtype=dt,buffer=m.data,strides=(m.row_step,m.point_step))
    p=np.c_[a['x'].ravel(),a['y'].ravel(),a['z'].ravel()].astype(float)
    return p[np.isfinite(p).all(1)]


class RosNode:
    def __init__(self,M,args):
        import rospy,tf2_ros
        from std_msgs.msg import String
        from geometry_msgs.msg import PoseStamped,TransformStamped
        from sensor_msgs.msg import PointCloud2
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage
        from sensor_msgs import point_cloud2
        from std_msgs.msg import Header
        self.rospy=rospy;self.String=String;self.Pose=PoseStamped;self.Transform=TransformStamped
        self.M,self.args=M,args;self.lock=threading.RLock();self.ctl=StartupController(M.cfg,getattr(args,'startup_started',time.monotonic()))
        self.tf={};self.legacy_warned=False;self.clouds=[];self.worker=None;self.exit_code=0;self.last_status=None;self.status_time=0.;self.last_row=None
        self.pub_pose=rospy.Publisher('/drone_reloc/pose',PoseStamped,queue_size=10)
        self.pub_result=rospy.Publisher('/drone_reloc/result',String,queue_size=1,latch=True)
        self.pub_status=rospy.Publisher('/drone_reloc/status',String,queue_size=1,latch=True)
        self.pub_map=rospy.Publisher('/drone_reloc/map',PointCloud2,queue_size=1,latch=True)
        self.broadcaster=tf2_ros.TransformBroadcaster()
        self.subs=[rospy.Subscriber('/tf_static',TFMessage,self.on_tf,queue_size=10),
                   rospy.Subscriber('/Odometry',Odometry,self.on_odom,queue_size=50),
                   rospy.Subscriber('/cloud_registered',PointCloud2,self.on_cloud,queue_size=30,buff_size=16777216)]
        self.pub_map.publish(point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.now(),frame_id='drone_map'),M.P.astype('f4').tolist()))
        # ROS Timer stops when /clock freezes; freshness must use wall-clock receipt time.
        self.watchdog_stop=threading.Event()
        rospy.on_shutdown(self.on_shutdown)
        self.watchdog=threading.Thread(target=self.watch_inputs,daemon=True);self.watchdog.start()
        self.publish_status(force=True)

    def ready_tf(self):return ODOM_EDGE in self.tf and MOUNT_EDGE in self.tf

    def on_tf(self,msg):
        with self.lock:
            for tr in msg.transforms:
                raw=tr.header.frame_id.strip('/'),tr.child_frame_id.strip('/');key=init_edge(*raw)
                if key is None:continue
                if key!=raw and not self.legacy_warned:
                    self.legacy_warned=True;self.rospy.logwarn('旧帧名 %s->%s（10-21 改名前），按 %s->%s 读',*raw,*key)
                q,p=tr.transform.rotation,tr.transform.translation
                try:R=quat_R([q.x,q.y,q.z,q.w]);t=np.array([p.x,p.y,p.z])
                except ValueError:self.ctl.invalidate('初始化TF旋转非法',True);continue
                if not np.isfinite(t).all():self.ctl.invalidate('初始化TF平移非法',True);continue
                if key in self.tf and any(not np.allclose(a,b,atol=1e-9,rtol=0) for a,b in zip(self.tf[key],(R,t))):
                    self.ctl.invalidate('初始化TF改变，旧定位失效，需重启',True)
                else:self.tf[key]=(R,t)
            self.publish_status()

    def on_odom(self,m):
        with self.lock:
            if not self.ready_tf():return
            if m.header.frame_id.strip('/')!='camera_init' or m.child_frame_id.strip('/')!='body':
                self.ctl.invalidate('Odometry坐标系不符',True);return
            p,q=m.pose.pose.position,m.pose.pose.orientation;row=np.array([m.header.stamp.to_sec(),p.x,p.y,p.z,q.x,q.y,q.z,q.w]);self.last_row=row
            Rwc,twc=self.tf[ODOM_EDGE];Rib,tib=self.tf[MOUNT_EDGE]
            try:
                world=row.copy();world[1:4]=Rwc@row[1:4]+twc;world[4:8]=R_quat(Rwc@quat_R(row[4:8]))
            except ValueError:self.ctl.invalidate('Odometry姿态非法',True);return
            self.ctl.observe_odom(world,time.monotonic());self.ctl.tick(time.monotonic())
            if self.ctl.valid and not self.rospy.is_shutdown():
                pm,Rm=mapped_pose(self.ctl.result,Rwc,twc,Rib,tib,row)
                ps=self.Pose();ps.header.stamp=m.header.stamp;ps.header.frame_id='drone_map'
                ps.pose.position.x,ps.pose.position.y,ps.pose.position.z=pm
                ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w=R_quat(Rm)
                R,t=unpack(self.ctl.result);tf=self.Transform();tf.header.stamp=m.header.stamp
                tf.header.frame_id='drone_map';tf.child_frame_id=ODOM;tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=t
                tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=R_quat(R)
                self.broadcaster.sendTransform(tf);self.pub_pose.publish(ps)

    def on_cloud(self,m):
        with self.lock:
            if m.header.frame_id.strip('/')!='camera_init':self.ctl.invalidate('点云坐标系不符',True);return
            self.ctl.observe_cloud(m.header.stamp.to_sec(),time.monotonic())
            if self.ctl.state=='COLLECTING' and m.header.stamp.to_sec()>=self.ctl.start_stamp:
                try:self.clouds.append((m.header.stamp.to_sec(),cloud2_xyz(m)))
                except ValueError as exc:self.ctl.invalidate(str(exc),True)

    def publish_status(self,force=False):
        now=time.monotonic();s=self.ctl.status(self.rospy.Time.now().to_sec());key=(s['state'],s['why'],s['valid'],s['confirmations'])
        if force or key!=self.last_status or now-self.status_time>=1.:
            if self.last_status is not None and self.last_status[2] and not s['valid']:
                self.pub_result.publish(self.String(json.dumps(dict(schema_version=2,ok=False,confirmed=False,
                    stamp=s['stamp'],why=s['why']),ensure_ascii=False)))
            self.pub_status.publish(self.String(json.dumps(s,ensure_ascii=False)));self.last_status=key;self.status_time=now

    def watch_inputs(self):
        while not self.watchdog_stop.wait(.05):
            if self.rospy.is_shutdown():return
            self.on_tick(None)

    def on_tick(self,_):
        with self.lock:
            previous=(self.ctl.state,self.ctl.start_stamp)
            self.ctl.tick(time.monotonic())
            job=self.ctl.job(time.monotonic()) if self.worker is None or not self.worker.is_alive() else None
            if self.ctl.state=='COLLECTING' and previous!=(self.ctl.state,self.ctl.start_stamp):self.clouds=[]
            if job:
                chunks=[p for ts,p in self.clouds if job['start']<=ts<=job['end']+1e-6]
                self.clouds=[];cloud=np.concatenate(chunks) if chunks else np.zeros((0,3))
                Rwc,twc=self.tf[ODOM_EDGE];cloud=cloud@Rwc.T+twc
                self.worker=threading.Thread(target=self.solve,args=(job,cloud),daemon=True);self.worker.start()
            self.publish_status()
            if self.ctl.terminal and self.ctl.state=='FAILED':
                self.exit_code=1;self.rospy.signal_shutdown(self.ctl.why)

    def solve(self,job,cloud):
        try:r=relocalize(self.M,cloud,job['center'],hint=job.get('hint'))
        except Exception as exc:r=dict(schema_version=2,ok=False,why='匹配异常：'+str(exc))
        with self.lock:
            if self.rospy.is_shutdown():return
            accepted=self.ctl.submit(r,job['token'],time.monotonic())
            geom_ok=r.get('ok',False)
            r.update(stamp=self.rospy.Time.now().to_sec(),map=self.args.map,confirmed=accepted,candidate_ok=geom_ok,ok=accepted)
            if geom_ok and not accepted:r['why']=self.ctl.why
            self.pub_result.publish(self.String(json.dumps(r,ensure_ascii=False)))
            self.publish_status(force=True)
            print('搜索%d: %s；确认%d/%d；%s'%(self.ctl.attempts,'可信候选' if geom_ok else '拒绝',self.ctl.confirmed,self.M.cfg['confirmations'],r.get('why','')),flush=True)
            if accepted and self.last_row is not None:
                # Same chain as /drone_reloc/pose: drone_base (origin = lidar IMU) in drone_map, from the latest odometry.
                Rwc,twc=self.tf[ODOM_EDGE];Rib,tib=self.tf[MOUNT_EDGE]
                pm,Rm=mapped_pose(self.ctl.result,Rwc,twc,Rib,tib,self.last_row)
                print('定位成功，当前位置（地图系，原点=建图起点的雷达 IMU）：x=%.3f y=%.3f z=%.3f m，航向 %.1f°，离地图地面 %.2f m'%(
                      pm[0],pm[1],pm[2],math.degrees(yaw(Rm)),pm[2]-self.M.zf),flush=True)
            if accepted and self.args.save:
                os.makedirs(self.args.save,exist_ok=True)
                fn=os.path.join(self.args.save,time.strftime('drone_reloc_%Y%m%d_%H%M%S.json'))
                with open(fn,'w') as f:json.dump(r,f,ensure_ascii=False,indent=2)

    def on_shutdown(self):
        self.watchdog_stop.set()
        with self.lock:
            if not self.ctl.terminal:self.ctl.invalidate('节点退出，输出失效',True)
            self.publish_status(force=True)


def online(M,args):
    import rospy
    rospy.init_node('drone_reloc',anonymous=False)
    node=RosNode(M,args);rospy.spin();return node.exit_code
