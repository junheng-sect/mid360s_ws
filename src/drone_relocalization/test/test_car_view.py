import math,os,tempfile,types,unittest
import numpy as np
from drone_relocalization.car_view import CarMap,CarViewNode,load_frames,se2,tf_yaw,HEADER,PX

CFG=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'config','map_frame.yaml')


def apply(T,x,y,yaw_deg=0.):
    p=T@np.array([x,y,0,1.]);return p[0],p[1],math.degrees(tf_yaw(T@se2(0,0,0,yaw_deg)))


class FakeBridge:
    def __init__(self,**latest):self.latest=latest
    def get(self,topic,max_age=None):return self.latest.get(topic)


class FrameTests(unittest.TestCase):
    def setUp(self):self.F=load_frames(CFG)

    def test_matches_registration(self):
        # 10-21 16:25 offline registration, car_map<-drone_map
        x,y,a=apply(self.F['car_drone'],0,0)
        self.assertAlmostEqual(x,.6803,delta=1e-3);self.assertAlmostEqual(y,.8924,delta=1e-3)
        self.assertAlmostEqual(a,-1.8151,delta=.01)

    def test_charging_pile_is_origin_and_docked_car_faces_x(self):
        # car app points in car_map: 充电桩 is the origin, 初始点 (car on the pile) faces map +x
        x,y,_=apply(self.F['map_car'],-0.5970993,-0.1469558)
        self.assertLess(math.hypot(x,y),1e-3)
        x,y,a=apply(self.F['map_car'],-0.6099617,-0.3682705,88.715996)
        self.assertAlmostEqual(x,-.222,delta=3e-3);self.assertAlmostEqual(y,.008,delta=3e-3);self.assertAlmostEqual(a,0,delta=.01)

    def test_floor_is_z0(self):
        self.assertAlmostEqual((self.F['map_drone']@np.array([0,0,-.1589,1.]))[2],0,delta=1e-6)

    def test_unknown_key_rejected(self):
        with open(CFG) as f:text=f.read()
        with tempfile.NamedTemporaryFile('w',suffix='.yaml',delete=False) as f:f.write(text+'extra: 1\n')
        try:
            with self.assertRaises(ValueError):load_frames(f.name)
        finally:os.unlink(f.name)


class CarStateTests(unittest.TestCase):
    def setUp(self):self.F=load_frames(CFG)
    def state(self,**latest):return CarViewNode.car_state(types.SimpleNamespace(bridge=FakeBridge(**latest),F=self.F))
    def pose(self):return {'pose':{'position':{'x':1,'y':2,'z':0},'orientation':{'x':0,'y':0,'z':0,'w':1}}}

    def test_calibrated_map(self):
        pose,why=self.state(**{'/tracked_pose':self.pose(),'/current_floormap':{'mapId':self.F['car_map_id']}})
        self.assertIsNotNone(pose);self.assertEqual(why,'')

    def test_other_map_withholds_car(self):
        pose,why=self.state(**{'/tracked_pose':self.pose(),'/current_floormap':{'mapId':123,'mapName':'old'}})
        self.assertIsNone(pose);self.assertIn('not the calibrated',why)

    def test_unknown_map_and_offline(self):
        self.assertEqual(self.state(**{'/tracked_pose':self.pose()}),(None,'car map id unknown'))
        self.assertEqual(self.state(),(None,'car offline'))


class RenderTests(unittest.TestCase):
    def test_size_and_car_pixels(self):
        F=load_frames(CFG);W,H=140,83
        grid={'info':{'width':W,'height':H,'resolution':.05,'origin':{'position':{'x':-3.65,'y':-1.284}}},'data':[0]*(W*H)}
        m=CarMap(grid);img=m.render(F,car=(1.,.5,0.),drone=(0.,1.,.7,0.),text='car ok')
        self.assertEqual(img.shape,(H*PX+HEADER,W*PX,3))
        u,v=m.px(1.+.325,.5)  # front edge of the body box, orange (0,140,255)
        self.assertTrue((abs(img[v-3:v+4,u-3:u+4].astype(int)-[0,140,255]).sum(2)<60).any())
        u,v=m.px(0.,1.)  # drone disc, magenta
        self.assertTrue((abs(img[v,u].astype(int)-[200,0,200])<30).all())
        self.assertEqual(m.px(-3.65,-1.284),(0,HEADER+H*PX))


if __name__=='__main__':
    unittest.main()
