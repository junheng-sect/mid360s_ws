"""Historical raw bag reader, no ROS master or hardware required."""
import json
import math
import os
import struct
import numpy as np
from drone_mapping import assemble
from .frames import ODOM_EDGE,MOUNT_EDGE,init_edge
from .geometry import quat_R,rotz,yaw,transform,unpack


class BagSession:
    def __init__(self,path):
        import rosbag
        self.path=path;self.bag=rosbag.Bag(path);od=[];transforms={};stamps=[]
        for topic,m,_ in self.bag.read_messages(topics=['/Odometry','/tf_static']):
            if topic=='/Odometry':
                if m.header.frame_id.strip('/')!='camera_init' or m.child_frame_id.strip('/')!='body':raise ValueError('Odometry frame mismatch')
                p,q=m.pose.pose.position,m.pose.pose.orientation
                od.append([m.header.stamp.to_sec(),p.x,p.y,p.z,q.x,q.y,q.z,q.w])
            else:
                for tr in m.transforms:
                    a,b=tr.header.frame_id.strip('/'),tr.child_frame_id.strip('/');q,p=tr.transform.rotation,tr.transform.translation
                    val=(np.array([q.x,q.y,q.z,q.w]),np.array([p.x,p.y,p.z]))
                    key=init_edge(a,b) or (a,b)  # bags from before 10-21 use world / base_link
                    if key in transforms and any(not np.allclose(x,y,atol=1e-10,rtol=0) for x,y in zip(val,transforms[key])):raise ValueError('bag initialization TF changes')
                    transforms[key]=val
        if len(od)<2 or ODOM_EDGE not in transforms or MOUNT_EDGE not in transforms:raise ValueError('bag needs Odometry, drone_odom->camera_init and body->drone_base (or the old world / base_link)')
        self.od=np.array(od,float)
        if not np.isfinite(self.od).all() or (np.diff(self.od[:,0])<=0).any():raise ValueError('invalid/backwards odometry timestamps')
        self.Rwc=quat_R(transforms[ODOM_EDGE][0]);self.twc=transforms[ODOM_EDGE][1]
        self.Rib=quat_R(transforms[MOUNT_EDGE][0]);self.tib=transforms[MOUNT_EDGE][1]
        self.t0=self.od[0,0];self.dur=self.od[-1,0]-self.t0
        self.Tli=np.array([-.011,-.02329,.04412]);self.Rli=np.eye(3);self.every=3;self.blind=.5
        pp=os.path.join(os.path.dirname(os.path.abspath(path)),'params.yaml')
        if os.path.isfile(pp):
            import yaml
            with open(pp) as f:p=yaml.safe_load(f) or {}
            mp=p.get('mapping',{});self.Tli=np.array(mp.get('extrinsic_T',self.Tli),float);self.Rli=np.array(mp.get('extrinsic_R',self.Rli),float).reshape(3,3)
            self.every=int(p.get('point_filter_num',3));self.blind=float(p.get('preprocess',{}).get('blind',.5))

    def close(self):self.bag.close()

    def pose(self,t):
        od=self.od;j=int(np.clip(np.searchsorted(od[:,0],t)-1,0,len(od)-2));a=float(np.clip((t-od[j,0])/(od[j+1,0]-od[j,0]),0,1))
        r=assemble.slerp_R(od[j:j+1,4:8],od[j+1:j+2,4:8],np.array([a]))[0]
        p=(1-a)*od[j,1:4]+a*od[j+1,1:4]
        return np.r_[t,p,self._quat(r)]

    @staticmethod
    def _quat(R):
        from .geometry import R_quat
        return R_quat(R)

    def pose_at(self,t):
        row=self.pose(t);return self.Rwc@row[1:4]+self.twc,yaw(self.Rwc@quat_R(row[4:8]))

    def cloud(self,t_rel,window):
        import rospy
        ta,tb=self.t0+t_rel,self.t0+t_rel+window;out=[]
        for _,raw,_ in self.bag.read_messages(topics=['/livox/lidar'],start_time=rospy.Time.from_sec(ta-.2),end_time=rospy.Time.from_sec(tb+.15),raw=True):
            d=raw[1];sc,ns=struct.unpack_from('<II',d,4);stamp=sc+ns*1e-9;off=32+struct.unpack_from('<I',d,12)[0];n=struct.unpack_from('<I',d,off)[0]
            a=np.frombuffer(d,assemble.PT,count=n,offset=off+4);tg=a['tag']&0x30
            a=a[(a['line']<4)&((tg==0)|(tg==0x10))][::self.every]
            tp=stamp+a['t']*1e-9;p=np.c_[a['x'],a['y'],a['z']].astype(float)
            ok=(tp>=max(ta,self.od[0,0]))&(tp<=min(tb,self.od[-1,0]))&(np.linalg.norm(p,axis=1)>self.blind)&np.isfinite(p).all(1)
            if ok.any():out.append(assemble.to_world(p[ok]@self.Rli.T+self.Tli,tp[ok],self.od,self.Rwc)+self.twc)
        return np.concatenate(out) if out else np.zeros((0,3))

    def lidar_stamps(self):
        out=[]
        for _,raw,_ in self.bag.read_messages(topics=['/livox/lidar'],raw=True):
            sc,ns=struct.unpack_from('<II',raw[1],4);out.append(sc+ns*1e-9)
        return np.array(out)


def run_once(M,S,t,window,perturb=0.):
    from .matching import relocalize
    cloud=S.cloud(t,window);c,heading=S.pose_at(S.t0+t+window/2)
    Rp=rotz(perturb)
    if perturb:cloud=(cloud-c)@Rp.T+c
    r=relocalize(M,cloud,c);r.update(t=float(t),n_pts=len(cloud),drone_world=c.tolist(),yaw_world=heading,perturb=perturb)
    if perturb and 'transform' in r:
        R,tr=unpack(r);r['transform']=transform(R@Rp,tr+R@(c-Rp@c));R,tr=unpack(r);r['T']=[*tr.tolist(),yaw(R)]
    return r
