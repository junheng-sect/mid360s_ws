#!/usr/bin/env python3
"""Laptop-side helpers for the reloc RViz view; runs in the docker container next to RViz, never on the board.

Reads the board's real TF (map, car_map, car_base, drone_base from car_bridge and relocalize), /car/status and
/drone_reloc/status, plus the car occupancy grid straight from the car's rosbridge (subscribe only).
  /reloc_display/car_map/compressed  2 Hz PNG of the car map drawn like the car app, with map axes, car body, drone
  /reloc_display/labels              text labels on map, car_base, drone_base
  /reloc_display/status_text         relocalization state
Images and markers only go to the RViz on this machine; the board just sees one more subscriber of its topics.
"""
import argparse
import json
import math
import sys
import threading
import time
sys.dont_write_bytecode=True
import cv2  # noqa: E402
import numpy as np  # noqa: E402

STATUS_STALE_S=2.5  # both status topics are re-published at >= 1 Hz
TF_STALE_S=1.  # car_base and FAST-LIO poses come at 10 Hz
BODY=(.65,.55)  # car body length x width (m), spec sheet, centred on base_footprint
PX=8  # image pixels per car map cell
HEADER=44


def yaw(T):
    return math.atan2(T[1,0],T[0,0])


def matrix(tr):
    q,t=tr.transform.rotation,tr.transform.translation;x,y,z,w=q.x,q.y,q.z,q.w
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),t.x],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),t.y],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y),t.z],[0,0,0,1.]])


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

    def render(self,origin=None,car=None,drone=None,text='',warn=False):
        """origin: car_map<-map (4x4); car: (x, y, yaw); drone: (x, y, height above floor, yaw); all in car_map."""
        img=np.vstack([np.full((HEADER,self.base.shape[1],3),40,np.uint8),self.base.copy()])
        if origin is not None:
            o=origin[:2,3];a=yaw(origin)
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


class GridFeed(threading.Thread):
    """Latest /map from the car rosbridge; subscribe only, reconnect every 2 s."""

    def __init__(self,url):
        super().__init__(daemon=True);self.url=url;self.lock=threading.Lock();self.grid=None;self.stop=threading.Event()

    def get(self):
        with self.lock:return self.grid

    def run(self):
        import websocket
        while not self.stop.is_set():
            ws=None
            try:
                ws=websocket.create_connection(self.url,timeout=3)
                ws.send(json.dumps(dict(op='subscribe',topic='/map',throttle_rate=5000,queue_length=1)))
                while not self.stop.is_set():
                    try:m=json.loads(ws.recv())
                    except websocket.WebSocketTimeoutException:continue
                    if m.get('op')=='publish':
                        with self.lock:self.grid=m['msg']
            except Exception:  # noqa: BLE001  car offline, rosbridge restarting, malformed frame
                pass
            finally:
                try:ws.close()
                except Exception:pass  # noqa: BLE001
            self.stop.wait(2.)


def status_text(s):
    """(text, rgb) for the relocalization state; ASCII only, RViz's text font has no CJK glyphs."""
    if s is None:return 'NO /drone_reloc/status - not localized',(1.,.25,.2)
    if s.get('valid'):return 'LOCALIZED - scan aligned to map',(.2,.9,.2)
    state=str(s.get('state','?'))
    return '%s - not localized, drone not shown'%state,((1.,.25,.2) if state in ('STALE','FAILED') else (1.,.8,.1))


class DisplayNode:
    def __init__(self,url):
        import rospy,tf2_ros
        from std_msgs.msg import String
        from sensor_msgs.msg import CompressedImage
        from visualization_msgs.msg import Marker
        self.rospy=rospy;self.Marker=Marker;self.Image=CompressedImage
        self.buf=tf2_ros.Buffer();self.listener=tf2_ros.TransformListener(self.buf)
        self.lock=threading.Lock();self.status={};self.grid=None;self.grid_key=None
        self.pub_label=rospy.Publisher('/reloc_display/labels',Marker,queue_size=4)
        self.pub_text=rospy.Publisher('/reloc_display/status_text',Marker,queue_size=1)
        self.pub_image=rospy.Publisher('/reloc_display/car_map/compressed',CompressedImage,queue_size=1)
        self.subs=[rospy.Subscriber(t,String,self.on_status,callback_args=t,queue_size=5) for t in ('/car/status','/drone_reloc/status')]
        self.feed=GridFeed(url);self.feed.start();rospy.on_shutdown(self.feed.stop.set)
        self.timer=rospy.Timer(rospy.Duration(.5),self.tick)

    def on_status(self,m,topic):
        try:s=json.loads(m.data)
        except ValueError:return
        with self.lock:self.status[topic]=(time.monotonic(),s)

    def fresh(self,topic):
        with self.lock:r=self.status.get(topic)
        return r[1] if r and time.monotonic()-r[0]<STATUS_STALE_S else None

    def lookup(self,target,source,stale=None):
        """target<-source 4x4, or None when missing or (for dynamic chains) older than `stale` s."""
        try:tr=self.buf.lookup_transform(target,source,self.rospy.Time(0))
        except Exception:return None  # noqa: BLE001  frame not published yet
        if stale is not None and tr.header.stamp.to_sec()>0 and self.rospy.Time.now().to_sec()-tr.header.stamp.to_sec()>stale:return None
        return matrix(tr)

    def marker(self,frame,ns,i,text,z,rgb,size=.2):
        m=self.Marker();m.header.frame_id=frame;m.header.stamp=self.rospy.Time(0);m.frame_locked=True
        m.ns=ns;m.id=i;m.type=m.TEXT_VIEW_FACING;m.action=m.ADD;m.text=text
        m.pose.position.z=z;m.pose.orientation.w=1.;m.scale.z=size
        m.color.r,m.color.g,m.color.b=rgb;m.color.a=1.
        return m

    def tick(self,_event):
        car_s=self.fresh('/car/status');drone_s=self.fresh('/drone_reloc/status')
        car_ok=bool(car_s and car_s.get('valid'));drone_ok=bool(drone_s and drone_s.get('valid'))
        car_why=('car_base' if car_ok else 'car (%s)'%(car_s.get('why') or 'not valid')) if car_s else 'car (no /car/status)'
        self.pub_label.publish(self.marker('map','reloc_display',0,'map (0,0,0)',.4,(.2,.9,1.)))
        # A label on a missing frame makes RViz complain; hang the car warning on map instead.
        self.pub_label.publish(self.marker('car_base' if car_ok else 'map','reloc_display',1,car_why,.35 if car_ok else .7,(1.,.55,0.) if car_ok else (1.,.25,.2)))
        m=self.marker('drone_base','reloc_display',2,'drone_base',.35,(.85,.2,.85))
        if not drone_ok:m.action=m.DELETE
        self.pub_label.publish(m)
        text,rgb=status_text(drone_s)
        self.pub_text.publish(self.marker('drone_base' if drone_ok else 'map','reloc_display_status',0,text,.7 if drone_ok else 1.,rgb,.35))
        self.publish_image(car_ok,car_s,car_why,drone_ok,drone_s)

    def publish_image(self,car_ok,car_s,car_why,drone_ok,drone_s):
        grid=self.feed.get()
        if grid is None:return
        info=grid['info'];key=(info['width'],info['height'],info['origin']['position']['x'],info['origin']['position']['y'],len(grid['data']),hash(tuple(grid['data'][::97])))
        if key!=self.grid_key:self.grid=CarMap(grid);self.grid_key=key
        origin=self.lookup('car_map','map');car=drone=None
        if car_ok:
            T=self.lookup('car_map','car_base',TF_STALE_S)
            if T is not None:car=(T[0,3],T[1,3],yaw(T))
        if drone_ok and origin is not None:
            T=self.lookup('car_map','drone_base',TF_STALE_S)
            if T is not None:drone=(T[0,3],T[1,3],T[2,3]-origin[2,3],yaw(T))  # height above the map floor (z=0)
        car_txt='car ok'+(' score %.2f'%car_s['score'] if car_s.get('score') is not None else '') if car_ok else car_why
        drone_txt='drone LOCALIZED' if drone else 'drone '+(str(drone_s.get('state','?')) if drone_s else 'no status')+' (not drawn)'
        img=self.grid.render(origin,car,drone,'%s | %s'%(car_txt,drone_txt),not car_ok)
        ok,buf=cv2.imencode('.png',img,[cv2.IMWRITE_PNG_COMPRESSION,3])
        if not ok:return
        m=self.Image();m.header.stamp=self.rospy.Time.now();m.header.frame_id='car_map'
        m.format='bgr8; png compressed bgr8';m.data=buf.tobytes();self.pub_image.publish(m)


def main():
    import rospy
    p=argparse.ArgumentParser(description='本机 RViz 显示辅助：二维车图、标签、定位状态（只读板端话题和车端 rosbridge）')
    p.add_argument('--car',default='192.168.31.231:9090',help='车端 rosbridge host:port（只取 /map 栅格画底图）')
    args=p.parse_args(rospy.myargv()[1:])
    rospy.init_node('reloc_display',anonymous=False)
    DisplayNode('ws://'+args.car);rospy.spin();return 0


if __name__=='__main__':
    try:sys.exit(main())
    except KeyboardInterrupt:sys.exit(130)
