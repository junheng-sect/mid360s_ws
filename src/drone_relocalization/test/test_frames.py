import os,tempfile,unittest
import numpy as np
from drone_relocalization.frames import ODOM_EDGE,MOUNT_EDGE,init_edge

NEW=[('drone_odom','camera_init'),('body','drone_base')]
OLD=[('world','camera_init'),('body','base_link')]  # bags recorded before the 10-21 rename


class InitEdgeTests(unittest.TestCase):
    def test_new_and_old_names(self):
        self.assertEqual(init_edge('drone_odom','camera_init'),ODOM_EDGE);self.assertEqual(init_edge('/world','camera_init'),ODOM_EDGE)
        self.assertEqual(init_edge('body','drone_base'),MOUNT_EDGE);self.assertEqual(init_edge('body','/base_link'),MOUNT_EDGE)
    def test_other_edges(self):
        for a,b in [('map','odom'),('world','body'),('base_footprint','base_link'),('camera_init','body'),('drone_base','drone_base_frd')]:
            self.assertIsNone(init_edge(a,b))


class BagNameTests(unittest.TestCase):
    """BagSession reads the gravity-bridge TF under either name and gets identical transforms."""
    def write(self,path,names):
        import rosbag,rospy
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage
        from geometry_msgs.msg import TransformStamped
        tf=TFMessage()
        for (a,b),(t,q) in zip(names,[((.1,.2,.3),(0,.1,0,.99498744)),((0,0,0),(0,-.16,0,.98711702))]):
            tr=TransformStamped();tr.header.frame_id=a;tr.child_frame_id=b
            tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=t
            tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w=q;tf.transforms.append(tr)
        with rosbag.Bag(path,'w') as bag:
            bag.write('/tf_static',tf,rospy.Time(100))
            for i in range(3):
                od=Odometry();od.header.stamp=rospy.Time(100+i*.1);od.header.frame_id='camera_init';od.child_frame_id='body'
                od.pose.pose.orientation.w=1.;bag.write('/Odometry',od,od.header.stamp)
    def load(self,names):
        from drone_relocalization.session import BagSession
        with tempfile.TemporaryDirectory() as d:
            p=os.path.join(d,'init.bag');self.write(p,names);S=BagSession(p)
            try:return S.Rwc,S.twc,S.Rib,S.tib
            finally:S.close()
    def test_old_and_new_names_match(self):
        new,old=self.load(NEW),self.load(OLD)
        for a,b in zip(new,old):np.testing.assert_array_equal(a,b)
        np.testing.assert_allclose(new[1],(.1,.2,.3))
    def test_unrelated_edges_rejected(self):
        with self.assertRaises(ValueError):self.load([('map','camera_init'),('body','base_footprint')])


if __name__=='__main__':unittest.main()
