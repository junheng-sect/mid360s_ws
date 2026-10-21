"""Car pose into the board's TF tree. Reads the M65 car's rosbridge only; never publishes to the car.

  /tf_static  map -> drone_map   map_frame.yaml; only when --drone-map is the calibrated drone map
              map -> car_map     map_frame.yaml
  /tf         car_map -> car_base  every /tracked_pose (10 Hz), z = car_base_height, board receive time
  /car/pose   PoseStamped, map <- car_base, same stamp as the TF
  /car/status latched JSON at 1 Hz: online, age_s, map_ok, score, why, valid
The car clock is not synced with the board, so stamps are the board's receive time (rosbridge throttle <= 0.1 s).
A car map id other than the calibrated one withholds car_base and /car/pose; status says why.
"""
import argparse
import importlib.util
import json
import os
import threading
import time
from . import calib

CAR_STALE_S=1.  # /tracked_pose is throttled to 10 Hz
TOPICS=(('/tracked_pose',100),('/current_floormap',2000),('/local_global_score',1000))


class Bridge(threading.Thread):
    """Latest message per topic from the car rosbridge; subscribe only, reconnect every 2 s.
    on_message(topic, msg, monotonic_receive_time) runs in this thread."""

    def __init__(self,url,on_message=None):
        super().__init__(daemon=True);self.url=url;self.on_message=on_message;self.lock=threading.Lock();self.latest={}
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
                        if self.on_message:self.on_message(m['topic'],m['msg'],last)
            except Exception:  # noqa: BLE001  car offline, rosbridge restarting, malformed frame
                pass
            finally:
                try:self.ws.close()
                except Exception:pass  # noqa: BLE001
            self.stop.wait(2.)


def car_state(bridge,F):
    """(tracked pose dict or None, reason); a pose only when the car is fresh and on the calibrated map."""
    floor=bridge.get('/current_floormap');pose=bridge.get('/tracked_pose',CAR_STALE_S)
    if pose is None:return None,'car offline'
    if floor is None:return None,'car map id unknown'
    if int(floor.get('mapId',-1))!=F['car_map_id']:
        return None,'car map %s (id %s) is not the calibrated %d'%(floor.get('mapName'),floor.get('mapId'),F['car_map_id'])
    return pose,''


def status(bridge,F,now):
    pose,why=car_state(bridge,F);floor=bridge.get('/current_floormap') or {};score=bridge.get('/local_global_score',5.)
    t=bridge.stamp('/tracked_pose')
    return dict(schema_version=1,online=bridge.get('/tracked_pose',CAR_STALE_S) is not None,age_s=round(now-t,3) if t else None,
                map_id=floor.get('mapId'),map_name=floor.get('mapName'),map_ok=floor.get('mapId')==F['car_map_id'],
                score=score['data'][-1] if score and score.get('data') else None,valid=pose is not None,why=why)


class CarBridgeNode:
    def __init__(self,F,drone_map,url):
        import rospy,tf2_ros
        from std_msgs.msg import String
        from geometry_msgs.msg import PoseStamped,TransformStamped
        self.rospy=rospy;self.String=String;self.Pose=PoseStamped;self.Transform=TransformStamped;self.F=F
        self.static=tf2_ros.StaticTransformBroadcaster();self.dynamic=tf2_ros.TransformBroadcaster()
        self.pub_pose=rospy.Publisher('/car/pose',PoseStamped,queue_size=5)
        self.pub_status=rospy.Publisher('/car/status',String,queue_size=1,latch=True)
        static=[self.tf('map','car_map',F['map_car'],rospy.Time.now())]
        name=os.path.basename(os.path.normpath(os.path.expanduser(drone_map)))
        if name==F['drone_map']:static.append(self.tf('map','drone_map',F['map_drone'],rospy.Time.now()))
        else:rospy.logerr('car_bridge: 无人机图 %s 不是标定用的 %s，不发 map->drone_map'%(name,F['drone_map']))
        self.static.sendTransform(static)
        self.bridge=Bridge(url,self.on_message)
        if importlib.util.find_spec('websocket') is None:rospy.logerr('car_bridge: 缺 websocket-client（pip3 install --user websocket-client），连不了车')
        else:self.bridge.start()
        rospy.on_shutdown(self.bridge.stop.set)
        self.timer=rospy.Timer(rospy.Duration(1.),self.publish_status)
        self.last_why=None;self.publish_status()

    def tf(self,parent,child,T,stamp):
        m=self.Transform();m.header.stamp=stamp;m.header.frame_id=parent;m.child_frame_id=child
        m.transform.translation.x,m.transform.translation.y,m.transform.translation.z=T[:3,3]
        m.transform.rotation.x,m.transform.rotation.y,m.transform.rotation.z,m.transform.rotation.w=calib.quat_yaw(calib.yaw(T))
        return m

    def on_message(self,topic,msg,_received):
        if topic!='/tracked_pose' or self.rospy.is_shutdown():return
        pose,why=car_state(self.bridge,self.F)
        if why!=self.last_why:self.last_why=why;self.publish_status()
        if pose is None:return
        stamp=self.rospy.Time.now();T=calib.car_base(self.F,pose)
        self.dynamic.sendTransform(self.tf('car_map','car_base',T,stamp))
        M=self.F['map_car']@T;ps=self.Pose();ps.header.stamp=stamp;ps.header.frame_id='map'
        ps.pose.position.x,ps.pose.position.y,ps.pose.position.z=M[:3,3]
        ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w=calib.quat_yaw(calib.yaw(M))
        self.pub_pose.publish(ps)

    def publish_status(self,_event=None):
        s=status(self.bridge,self.F,time.monotonic());s['stamp']=self.rospy.Time.now().to_sec()
        self.pub_status.publish(self.String(data=json.dumps(s,ensure_ascii=False)))


def main():
    import rospy
    p=argparse.ArgumentParser(description='车位姿进板端 TF：只读车端 rosbridge')
    p.add_argument('--drone-map',required=True,help='无人机先验地图目录或名字，用来核对 map->drone_map 标定')
    p.add_argument('--frames',required=True,help='config/map_frame.yaml')
    p.add_argument('--car',default='192.168.31.231:9090',help='车端 rosbridge host:port')
    args=p.parse_args(rospy.myargv()[1:])
    rospy.init_node('car_bridge',anonymous=False)
    CarBridgeNode(calib.load(args.frames),args.drone_map,'ws://'+args.car)
    rospy.spin();return 0
