"""Car and unified map in the RViz comparison view, display only.

The M65 car has its own ROS master; this node only reads it through the car's rosbridge and never publishes
to it. Frames, display-only like reloc_view's drone_map_view (not the real /tf map/car_map):
  drone_map_view -> map_view      static, inverse of map<-drone_map from config/map_frame.yaml
  map_view -> car_map_view        static, map<-car_map
  car_map_view -> car_view        /tracked_pose x, y, yaw at a fixed height above the floor
Also text labels and a 2D picture of the car map (map axes, car body, localized drone) as a compressed
image for an RViz Image panel. A car map id or drone map name other than the calibrated one withholds the
frames that depend on it.
"""
import argparse
import importlib.util
import json
import math
import os
import threading
import time
import cv2
import numpy as np
import yaml

CAR_STALE_S=2.  # /tracked_pose is throttled to 10 Hz
DRONE_STALE_S=1.;STATUS_STALE_S=2.5  # same status rule as reloc_view
BODY=(.65,.55)  # car body length x width (m), spec sheet, centred on base_footprint
PX=8  # image pixels per car map cell
HEADER=44
TOPICS=(('/tracked_pose',100),('/map',5000),('/current_floormap',2000),('/local_global_score',1000))


def se2(x,y,z,yaw_deg):
    a=math.radians(yaw_deg);c,s=math.cos(a),math.sin(a)
    return np.array([[c,-s,0,x],[s,c,0,y],[0,0,1,z],[0,0,0,1.]])


def tf_yaw(T):
    return math.atan2(T[1,0],T[0,0])


def load_frames(path):
    with open(os.path.expanduser(path)) as f:d=yaml.safe_load(f)
    keys={'drone_map','car_map_id','map_from_drone_map','map_from_car_map','car_frame_height'}
    if not isinstance(d,dict) or set(d)!=keys:raise ValueError('map_frame keys must be exactly %s'%sorted(keys))
    T={k:se2(*(float(d[k][a]) for a in ('x','y','z','yaw_deg'))) for k in ('map_from_drone_map','map_from_car_map')}
    return dict(drone_map=str(d['drone_map']),car_map_id=int(d['car_map_id']),height=float(d['car_frame_height']),
                map_drone=T['map_from_drone_map'],map_car=T['map_from_car_map'],
                car_drone=np.linalg.inv(T['map_from_car_map'])@T['map_from_drone_map'])


def pose_xyyaw(msg):
    p,q=msg['pose']['position'],msg['pose']['orientation']
    return p['x'],p['y'],math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))


class CarMap:
    """Car occupancy grid drawn like the car app: car_map x right, y up."""

    def __init__(self,grid):
        info=grid['info'];self.W,self.H=int(info['width']),int(info['height'])
        self.res=float(info['resolution']);o=info['origin']['position'];self.ox,self.oy=float(o['x']),float(o['y'])
        v=np.array(grid['data'],np.int16).reshape(self.H,self.W)[::-1]
        g=np.where(v<0,205,np.clip(255-v*2.15,40,255)).astype(np.uint8)  # unknown grey, free white, occupied dark
        self.base=cv2.cvtColor(np.kron(g,np.ones((PX,PX),np.uint8)),cv2.COLOR_GRAY2BGR)

    def px(self,x,y):
        return (int(round((x-self.ox)/self.res*PX)),int(round(HEADER+(self.H-(y-self.oy)/self.res)*PX)))

    def render(self,F,car=None,drone=None,text='',warn=False):
        img=np.vstack([np.full((HEADER,self.base.shape[1],3),40,np.uint8),self.base.copy()])
        Tcm=np.linalg.inv(F['map_car']);o=Tcm[:2,3];a=tf_yaw(Tcm)
        for ang,color,name in ((a,(0,0,230),'x'),(a+math.pi/2,(0,170,0),'y')):
            tip=o+.5*np.array([math.cos(ang),math.sin(ang)])
            cv2.arrowedLine(img,self.px(*o),self.px(*tip),color,3,cv2.LINE_AA,tipLength=.2)
            cv2.putText(img,name,self.px(*(tip+.08*np.array([math.cos(ang),math.sin(ang)]))),cv2.FONT_HERSHEY_SIMPLEX,.7,color,2,cv2.LINE_AA)
        cv2.putText(img,'map',self.px(o[0]+.08,o[1]-.22),cv2.FONT_HERSHEY_SIMPLEX,.7,(160,120,0),2,cv2.LINE_AA)
        if car:
            x,y,th=car;c,s=math.cos(th),math.sin(th);L,W=BODY[0]/2,BODY[1]/2
            pts=[self.px(x+c*u-s*v,y+s*u+c*v) for u,v in ((L,W),(-L,W),(-L,-W),(L,-W))]
            cv2.polylines(img,[np.array(pts,np.int32)],True,(0,140,255),3,cv2.LINE_AA)
            cv2.arrowedLine(img,self.px(x,y),self.px(x+.45*c,y+.45*s),(0,140,255),3,cv2.LINE_AA,tipLength=.25)
            cv2.putText(img,'car',self.px(x-L,y+W+.06),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,140,255),2,cv2.LINE_AA)
        if drone:
            x,y,z,th=drone
            cv2.circle(img,self.px(x,y),int(.12/self.res*PX),(200,0,200),-1,cv2.LINE_AA)
            cv2.line(img,self.px(x,y),self.px(x+.35*math.cos(th),y+.35*math.sin(th)),(200,0,200),3,cv2.LINE_AA)
            cv2.putText(img,'drone z=%.2fm'%z,self.px(x+.15,y-.3),cv2.FONT_HERSHEY_SIMPLEX,.7,(200,0,200),2,cv2.LINE_AA)
        cv2.putText(img,text,(12,30),cv2.FONT_HERSHEY_SIMPLEX,.75,(80,80,255) if warn else (235,235,235),2,cv2.LINE_AA)
        return img


class Bridge(threading.Thread):
    """Latest message per topic from the car rosbridge; subscribe only, reconnect every 2 s."""

    def __init__(self,url):
        super().__init__(daemon=True);self.url=url;self.lock=threading.Lock();self.latest={}
        self.stop=threading.Event();self.ws=None

    def get(self,topic,max_age=None):
        with self.lock:r=self.latest.get(topic)
        if r is None or max_age is not None and time.monotonic()-r[0]>max_age:return None
        return r[1]

    def stamp(self,topic):
        with self.lock:return self.latest.get(topic,(0.,None))[0]

    def run(self):
        import websocket
        while not self.stop.is_set():
            try:
                self.ws=ws=websocket.create_connection(self.url,timeout=3)
                for topic,ms in TOPICS:ws.send(json.dumps(dict(op='subscribe',topic=topic,throttle_rate=ms,queue_length=1)))
                last=time.monotonic()
                while not self.stop.is_set() and time.monotonic()-last<5:  # tracked_pose comes at 10 Hz
                    try:m=json.loads(ws.recv())
                    except websocket.WebSocketTimeoutException:continue
                    if m.get('op')=='publish':
                        last=time.monotonic()
                        with self.lock:self.latest[m['topic']]=(last,m['msg'])
            except Exception:  # noqa: BLE001  car offline, rosbridge restarting, malformed frame
                pass
            finally:
                try:self.ws.close()
                except Exception:pass  # noqa: BLE001
            self.stop.wait(2.)


class CarViewNode:
    def __init__(self,F,map_dir,url):
        import rospy,tf2_ros
        from std_msgs.msg import String
        from sensor_msgs.msg import CompressedImage
        from geometry_msgs.msg import PoseStamped,TransformStamped
        from visualization_msgs.msg import Marker
        self.rospy=rospy;self.Marker=Marker;self.Transform=TransformStamped;self.Image=CompressedImage;self.F=F
        self.lock=threading.Lock();self.status=None;self.status_time=0.;self.drone=None;self.drone_time=0.
        self.grid=None;self.grid_key=None;self.car_sent=0.;self.tick_n=0
        self.map_ok=os.path.basename(os.path.normpath(map_dir))==F['drone_map']
        if not self.map_ok:rospy.logerr('car_view: 无人机图 %s 不是标定用的 %s，不发 map_view/car 坐标'%(map_dir,F['drone_map']))
        self.static=tf2_ros.StaticTransformBroadcaster();self.dynamic=tf2_ros.TransformBroadcaster()
        self.pub_label=rospy.Publisher('/car_view/labels',Marker,queue_size=4)
        self.pub_image=rospy.Publisher('/car_view/map_image/compressed',CompressedImage,queue_size=1)
        self.subs=[rospy.Subscriber('/drone_reloc/status',String,self.on_status,queue_size=5),
                   rospy.Subscriber('/drone_reloc/pose',PoseStamped,self.on_pose,queue_size=5)]
        if self.map_ok:
            Tdm=np.linalg.inv(F['map_drone'])
            self.static.sendTransform([self.tf('drone_map_view','map_view',Tdm),self.tf('map_view','car_map_view',F['map_car'])])
        self.bridge=Bridge(url)
        if importlib.util.find_spec('websocket') is None:rospy.logerr('car_view: 缺 websocket-client（pip3 install --user websocket-client），只显示 map')
        else:self.bridge.start()
        rospy.on_shutdown(self.bridge.stop.set)
        self.timer=rospy.Timer(rospy.Duration(.1),self.tick)

    def tf(self,parent,child,T):
        from .geometry import R_quat
        m=self.Transform();m.header.stamp=self.rospy.Time.now();m.header.frame_id=parent;m.child_frame_id=child
        m.transform.translation.x,m.transform.translation.y,m.transform.translation.z=T[:3,3]
        m.transform.rotation.x,m.transform.rotation.y,m.transform.rotation.z,m.transform.rotation.w=R_quat(T[:3,:3])
        return m

    def on_status(self,m):
        try:s=json.loads(m.data)
        except ValueError:return
        with self.lock:self.status=s;self.status_time=time.monotonic()

    def on_pose(self,m):
        p,q=m.pose.position,m.pose.orientation
        with self.lock:self.drone=(p.x,p.y,p.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y**2+q.z**2)));self.drone_time=time.monotonic()

    def car_state(self):
        """(pose or None, reason); pose only when the car is fresh and on the calibrated map."""
        floor=self.bridge.get('/current_floormap');pose=self.bridge.get('/tracked_pose',CAR_STALE_S)
        if pose is None:return None,'car offline'
        if floor is None:return None,'car map id unknown'
        if int(floor.get('mapId',-1))!=self.F['car_map_id']:
            return None,'car map %s (id %s) is not the calibrated %d'%(floor.get('mapName'),floor.get('mapId'),self.F['car_map_id'])
        return pose,''

    def tick(self,_event):
        pose,why=self.car_state()
        if pose is not None and self.map_ok and self.bridge.stamp('/tracked_pose')!=self.car_sent:
            self.car_sent=self.bridge.stamp('/tracked_pose');x,y,th=pose_xyyaw(pose)
            self.dynamic.sendTransform(self.tf('car_map_view','car_view',se2(x,y,self.F['height'],math.degrees(th))))
        self.tick_n+=1
        if self.tick_n%5:return  # labels and picture at 2 Hz
        if self.map_ok:self.publish_labels(why)
        self.publish_image(pose,why)

    def label(self,frame,i,text,z,rgb):
        m=self.Marker();m.header.frame_id=frame;m.header.stamp=self.rospy.Time(0);m.frame_locked=True
        m.ns='car_view';m.id=i;m.type=m.TEXT_VIEW_FACING;m.action=m.ADD;m.text=text
        m.pose.position.z=z;m.pose.orientation.w=1.;m.scale.z=.2
        m.color.r,m.color.g,m.color.b=rgb;m.color.a=1.
        return m

    def publish_labels(self,why):
        # ASCII only: RViz's text font has no CJK glyphs. Cyan like the old drone map origin label.
        self.pub_label.publish(self.label('map_view',0,'map (0,0,0)',.4,(.2,.9,1.)))
        self.pub_label.publish(self.label('car_view',1,'car base_footprint' if not why else 'car (%s)'%why,.35,(1.,.55,0.) if not why else (1.,.25,.2)))
        # base_link only exists while FAST-LIO runs; a fresh reloc status is the proxy, otherwise drop the label
        with self.lock:alive=self.status is not None and time.monotonic()-self.status_time<STATUS_STALE_S
        m=self.label('base_link',2,'drone base_link',.35,(.85,.2,.85))
        if not alive:m.action=m.DELETE
        self.pub_label.publish(m)

    def publish_image(self,pose,why):
        grid=self.bridge.get('/map')
        if grid is None:return
        key=(grid['info']['width'],grid['info']['height'],grid['info']['origin']['position']['x'],grid['info']['origin']['position']['y'],len(grid['data']),hash(tuple(grid['data'][::97])))
        if key!=self.grid_key:self.grid=CarMap(grid);self.grid_key=key
        with self.lock:
            s=self.status if self.status is not None and time.monotonic()-self.status_time<STATUS_STALE_S else None
            d=self.drone if time.monotonic()-self.drone_time<DRONE_STALE_S else None
        drone=None
        if self.map_ok and s and s.get('valid') and d:
            p=self.F['car_drone']@np.array([d[0],d[1],d[2],1.])
            drone=(p[0],p[1],d[2]+self.F['map_drone'][2,3],d[3]+tf_yaw(self.F['car_drone']))
        score=self.bridge.get('/local_global_score',5.)
        car_txt=why or 'car ok'+(' score %.2f'%score['data'][-1] if score and score.get('data') else '')
        drone_txt='drone LOCALIZED' if drone else 'drone '+(str(s.get('state','?')) if s else 'no status')+' (not drawn)'
        img=self.grid.render(self.F,pose_xyyaw(pose) if pose is not None else None,drone,
                             '%s | %s%s'%(car_txt,drone_txt,'' if self.map_ok else ' | drone map mismatch'),bool(why) or not self.map_ok)
        ok,buf=cv2.imencode('.png',img,[cv2.IMWRITE_PNG_COMPRESSION,3])
        if not ok:return
        m=self.Image();m.header.stamp=self.rospy.Time.now();m.header.frame_id='car_map_view'
        m.format='bgr8; png compressed bgr8';m.data=buf.tobytes();self.pub_image.publish(m)


def main():
    import rospy
    p=argparse.ArgumentParser(description='RViz 显示用：小车坐标、统一 map 和二维车图（只读车端 rosbridge）')
    p.add_argument('--map',required=True,help='无人机先验地图目录，用来核对标定')
    p.add_argument('--frames',required=True,help='config/map_frame.yaml')
    p.add_argument('--car',default='192.168.31.231:9090',help='车端 rosbridge host:port')
    args=p.parse_args(rospy.myargv()[1:])
    rospy.init_node('car_view',anonymous=False)
    CarViewNode(load_frames(args.frames),os.path.expanduser(args.map),'ws://'+args.car);rospy.spin();return 0
