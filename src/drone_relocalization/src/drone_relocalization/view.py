"""RViz comparison view, display only.

The prior map is re-published in its own frame drone_map_view, child of drone_odom. Before a valid localization
drone_odom->drone_map_view is identity, i.e. the live cloud is drawn as if the start were the drone_map origin, so a drone
started elsewhere shows a visible offset. Once status is valid it becomes the inverse of drone_map<-drone_odom and the
live cloud lands on the map. The real /tf drone_map->drone_odom and /drone_reloc/pose are untouched; consumers must
still check status.valid.
"""
import json
import threading
import time
import numpy as np
from .frames import ODOM,BASE
from .geometry import R_quat,unpack

VIEW='drone_map_view'
STALE_S=2.5  # status is re-published at >=1 Hz; silence longer than this means the node died


class ViewNode:
    def __init__(self):
        import rospy,tf2_ros
        from std_msgs.msg import String
        from sensor_msgs.msg import PointCloud2
        from visualization_msgs.msg import Marker
        from geometry_msgs.msg import TransformStamped
        self.rospy=rospy;self.Marker=Marker;self.Transform=TransformStamped
        self.lock=threading.Lock();self.status=None;self.status_time=0.;self.result=None;self.sent=None;self.text=None
        self.broadcaster=tf2_ros.StaticTransformBroadcaster()
        self.pub_map=rospy.Publisher('/drone_reloc/view/map',PointCloud2,queue_size=1,latch=True)
        self.pub_text=rospy.Publisher('/drone_reloc/view/status_text',Marker,queue_size=1,latch=True)
        self.subs=[rospy.Subscriber('/drone_reloc/map',PointCloud2,self.on_map,queue_size=1,buff_size=16777216),
                   rospy.Subscriber('/drone_reloc/status',String,self.on_status,queue_size=5),
                   rospy.Subscriber('/drone_reloc/result',String,self.on_result,queue_size=5)]
        # Wall clock like runtime.watch_inputs: a frozen /clock must not keep a dead localization on screen.
        self.stop=threading.Event();rospy.on_shutdown(self.stop.set)
        self.tick();self.thread=threading.Thread(target=self.loop,daemon=True);self.thread.start()

    def on_map(self,m):
        m.header.frame_id=VIEW;self.pub_map.publish(m)

    def on_status(self,m):
        try:s=json.loads(m.data)
        except ValueError:return
        with self.lock:self.status=s;self.status_time=time.monotonic()

    def on_result(self,m):
        try:r=json.loads(m.data)
        except ValueError:return
        # Candidates before confirmation and the invalidation notice both carry ok=false.
        with self.lock:self.result=r if r.get('ok') and r.get('confirmed') and 'transform' in r else None

    def loop(self):
        while not self.stop.wait(.2):
            if self.rospy.is_shutdown():return
            self.tick()

    def tick(self):
        with self.lock:
            s=self.status if self.status is not None and time.monotonic()-self.status_time<STALE_S else None
            r=self.result if s and s.get('valid') else None
        Rv,tv=np.eye(3),np.zeros(3)
        if r:
            try:R,t=unpack(r);Rv,tv=R.T,-R.T@t  # drone_odom->view is the inverse of drone_map<-drone_odom
            except (ValueError,KeyError):r=None
        key=(r is not None,)+tuple(np.round(np.r_[tv,R_quat(Rv)],9))
        if key!=self.sent:
            tf=self.Transform();tf.header.stamp=self.rospy.Time.now();tf.header.frame_id=ODOM;tf.child_frame_id=VIEW
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=tv
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=R_quat(Rv)
            self.broadcaster.sendTransform(tf);self.sent=key
        self.publish_text(s,r is not None)

    def publish_text(self,s,aligned):
        # ASCII only: RViz's text font has no CJK glyphs.
        if aligned:text,rgb='LOCALIZED - scan aligned to map',(.2,.9,.2)
        elif s is None:text,rgb='NO /drone_reloc/status - not localized',(1.,.25,.2)
        else:
            state=str(s.get('state','?'))
            text='%s - not localized, start assumed at drone_map origin'%state
            rgb=(1.,.25,.2) if state in ('STALE','FAILED') else (1.,.8,.1)
        if (text,rgb)==self.text:return
        m=self.Marker();m.header.frame_id=BASE;m.header.stamp=self.rospy.Time(0);m.frame_locked=True
        m.ns='drone_reloc_view';m.id=0;m.type=m.TEXT_VIEW_FACING;m.action=m.ADD;m.text=text
        m.pose.position.z=.7;m.pose.orientation.w=1.;m.scale.z=.35
        m.color.r,m.color.g,m.color.b=rgb;m.color.a=1.
        self.pub_text.publish(m);self.text=(text,rgb)


def main():
    import rospy
    rospy.init_node('reloc_view',anonymous=False)
    ViewNode();rospy.spin();return 0
