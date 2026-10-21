import math,os,sys,tempfile,unittest
import numpy as np
sys.path.insert(0,os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'src'))
from car_bridge import calib  # noqa: E402
from car_bridge.node import car_state,status  # noqa: E402

CFG=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'config','map_frame.yaml')


def apply(T,x,y,yaw_deg=0.):
    p=T@np.array([x,y,0,1.]);return p[0],p[1],math.degrees(calib.yaw(T@calib.se2(0,0,0,yaw_deg)))


def pose(x=1.,y=2.,yaw=0.):
    return {'pose':{'position':{'x':x,'y':y,'z':0},'orientation':{'x':0,'y':0,'z':math.sin(yaw/2),'w':math.cos(yaw/2)}}}


class FakeBridge:
    def __init__(self,**latest):self.latest=latest
    def get(self,topic,max_age=None):return self.latest.get(topic)
    def stamp(self,topic):return 100. if topic in self.latest else 0.


class FrameTests(unittest.TestCase):
    def setUp(self):self.F=calib.load(CFG)

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

    def test_car_base_at_lidar_height(self):
        # docked car: car_base sits on the 初始点 in map, 0.20 m up, heading map +x
        M=self.F['map_car']@calib.car_base(self.F,pose(-0.6099617,-0.3682705,math.radians(88.715996)))
        self.assertAlmostEqual(M[2,3],.20,delta=1e-9);self.assertAlmostEqual(math.degrees(calib.yaw(M)),0,delta=.01)
        q=calib.quat_yaw(calib.yaw(M));self.assertAlmostEqual(np.linalg.norm(q),1,delta=1e-12)

    def test_unknown_key_rejected(self):
        with open(CFG) as f:text=f.read()
        with tempfile.NamedTemporaryFile('w',suffix='.yaml',delete=False) as f:f.write(text+'extra: 1\n')
        try:
            with self.assertRaises(ValueError):calib.load(f.name)
        finally:os.unlink(f.name)


class CarStateTests(unittest.TestCase):
    def setUp(self):self.F=calib.load(CFG)

    def test_calibrated_map(self):
        b=FakeBridge(**{'/tracked_pose':pose(),'/current_floormap':{'mapId':self.F['car_map_id'],'mapName':'10.21'},'/local_global_score':{'data':[.5,.9]}})
        p,why=car_state(b,self.F);self.assertIsNotNone(p);self.assertEqual(why,'')
        s=status(b,self.F,100.5);self.assertTrue(s['valid'] and s['online'] and s['map_ok']);self.assertEqual(s['score'],.9);self.assertEqual(s['age_s'],.5)

    def test_other_map_withholds_car(self):
        b=FakeBridge(**{'/tracked_pose':pose(),'/current_floormap':{'mapId':123,'mapName':'old'}})
        p,why=car_state(b,self.F);self.assertIsNone(p);self.assertIn('not the calibrated',why)
        s=status(b,self.F,101.);self.assertFalse(s['valid'] or s['map_ok']);self.assertTrue(s['online'])

    def test_unknown_map_and_offline(self):
        self.assertEqual(car_state(FakeBridge(**{'/tracked_pose':pose()}),self.F),(None,'car map id unknown'))
        self.assertEqual(car_state(FakeBridge(),self.F),(None,'car offline'))
        s=status(FakeBridge(),self.F,5.);self.assertFalse(s['online'] or s['valid']);self.assertIsNone(s['age_s'])


if __name__=='__main__':
    unittest.main()
