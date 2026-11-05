import json,math,os,tempfile,unittest
import numpy as np
from drone_relocalization.geometry import exp,quat_R,R_quat,transform,mapped_pose
from drone_relocalization.controller import StartupController
from drone_relocalization.settings import load
from drone_relocalization.matching import NNField,MapModel,relocalize,rank_hypotheses


def result(t=(0,0,0),rv=(0,0,0)):
    return dict(ok=True,transform=transform(exp(rv),t))


class ControllerTests(unittest.TestCase):
    def setUp(self):self.c=StartupController();self.now=0.
    def feed(self,duration=.1,p=(0,0,0)):
        self.now=round(self.now+duration,6);row=[100+self.now,*p,0,0,0,1]
        self.c.observe_odom(row,self.now);self.c.observe_cloud(100+self.now,self.now);self.c.tick(self.now)
    def job(self):
        for _ in range(80):
            self.feed();j=self.c.job(self.now)
            if j:return j
        self.fail('no job')
    def lock(self):
        for _ in range(2):
            j=self.job();self.c.submit(result(),j['token'],self.now)
        self.assertTrue(self.c.valid)
    def test_two_independent_windows(self):
        a=self.job();self.assertFalse(self.c.submit(result(),a['token'],self.now));b=self.job()
        self.assertGreater(b['start'],a['end']);self.c.submit(result(),b['token'],self.now);self.assertTrue(self.c.valid)
    def test_move_and_return_discards_search(self):
        j=self.job();self.feed(p=(.04,0,0));self.feed(p=(0,0,0))
        self.assertFalse(self.c.submit(result(),j['token'],self.now));self.assertFalse(self.c.valid)
    def test_move_and_return_during_collection(self):
        for _ in range(12):self.feed()
        self.assertEqual(self.c.state,'COLLECTING');epoch=self.c.epoch
        self.feed(p=(0,.04,0));self.feed();self.assertGreater(self.c.epoch,epoch);self.assertFalse(self.c.valid)
    def test_angular_motion_during_confirmation(self):
        j=self.job();self.c.submit(result(),j['token'],self.now);self.now+=.1
        self.c.observe_odom([100+self.now,0,0,0,*R_quat(exp([0,0,math.radians(3)]))],self.now)
        self.assertEqual(self.c.confirmed,0)
    def test_inconsistent_candidates_do_not_lock(self):
        a=self.job();self.c.submit(result(),a['token'],self.now);b=self.job()
        self.c.submit(result((.2,0,0)),b['token'],self.now);self.assertFalse(self.c.valid)
    def test_stale_after_lock_stops_output(self):
        self.lock();self.c.tick(self.now+.51);self.assertEqual(self.c.state,'STALE');self.assertIsNone(self.c.result)
        self.feed();self.assertFalse(self.c.valid)
    def test_time_backwards_is_terminal(self):
        self.lock();self.c.observe_odom([99,0,0,0,0,0,0,1],self.now+.01);self.assertEqual(self.c.state,'STALE')
    def test_cloud_only_stale_stops_output(self):
        self.lock()
        for _ in range(7):
            self.now+=.1;self.c.observe_odom([100+self.now,0,0,0,0,0,0,1],self.now);self.c.tick(self.now)
        self.assertFalse(self.c.valid);self.assertEqual(self.c.state,'STALE')
    def test_tf_change_policy_is_terminal(self):
        self.lock();self.c.invalidate('初始化TF改变',True);self.assertFalse(self.c.valid);self.assertEqual(self.c.state,'STALE')
    def test_ros_clock_freeze_uses_receipt_clock(self):
        self.lock();self.c.tick(self.now+1);self.assertFalse(self.c.valid)
    def test_60_second_deadline_without_data(self):
        self.c.tick(60);self.assertEqual(self.c.state,'FAILED')
    def test_three_failures_stop_searching(self):
        for _ in range(3):
            j=self.job();self.c.submit(dict(ok=False,why='几何不可信'),j['token'],self.now)
        self.assertEqual(self.c.attempts,3);self.assertTrue(self.c.terminal);self.assertFalse(self.c.valid)
    def test_old_worker_cannot_commit_after_deadline(self):
        j=self.job();self.c.submit(result(),j['token'],61);self.assertFalse(self.c.valid)
    def test_cloud_backwards_rejected(self):
        self.feed();self.c.observe_cloud(50,self.now);self.assertTrue(self.c.terminal)


class GeometryTests(unittest.TestCase):
    def test_pi_quaternion_roundtrip(self):
        for v in ([math.pi,0,0],[0,math.pi,0],[0,0,math.pi],[1,-2,.4]):
            R=exp(v);np.testing.assert_allclose(quat_R(R_quat(R)),R,atol=1e-8)
    def test_base_attitude_and_origin(self):
        r=result((1,2,3),(0,0,math.pi/2));Rwc=exp([0,.2,0]);Rib=exp([0,-.2,0])
        p,R=mapped_pose(r,Rwc,np.zeros(3),Rib,np.zeros(3),np.array([0,0,0,0,0,0,0,1]))
        np.testing.assert_allclose(p,[1,2,3],atol=1e-9);np.testing.assert_allclose(R,exp([0,0,math.pi/2]),atol=1e-9)
    def test_config_does_not_relax_gates(self):
        c=load();self.assertEqual(c['fit_ok'],.8);self.assertEqual(c['degen_ok'],.12);self.assertEqual(c['rmse_ok'],.02)
        with self.assertRaises(ValueError):load(overrides={'confirmations':1})


class FieldTests(unittest.TestCase):
    def test_cache_corrupt_and_fingerprint(self):
        x,y=np.meshgrid(np.arange(0,.5,.05),np.arange(0,.5,.05));P=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
        field=NNField(P)
        with tempfile.TemporaryDirectory() as d:
            p=os.path.join(d,'cache.npz');field.save(p,'a');self.assertIsNotNone(NNField.load(p,P,'a'));self.assertIsNone(NNField.load(p,P,'b'))
            with open(p,'wb') as f:f.write(b'PK\x03\x04truncated')
            self.assertIsNone(NNField.load(p,P,'a'))
    def test_degenerate_stronger_rival_cannot_promote_weaker_pose(self):
        # A long wall supports several offsets: the stronger degenerate match is evidence of ambiguity.
        weak=dict(R=np.eye(3),t=np.array([3.,0,0]),inlier=.444,support=.40,rmse=.010,degen=.123)
        strong=dict(R=np.eye(3),t=np.array([1.7,0,0]),inlier=.598,support=.50,rmse=.010,degen=.090)
        other=dict(R=np.eye(3),t=np.array([-3.9,0,0]),inlier=.566,support=.49,rmse=.012,degen=.085)
        trials,ambiguity=rank_hypotheses([(weak,.67),(strong,.67),(other,.65)])
        self.assertIs(trials[0][0],strong);self.assertLess(trials[0][0]['degen'],.12)
        self.assertGreater(ambiguity,.9)
    def test_low_residual_rival_is_still_ambiguous(self):
        # More hard inliers at a wrong pose cannot hide a competing better residual fit.
        a=dict(R=np.eye(3),t=np.zeros(3),inlier=.68,support=.55,rmse=.0126)
        b=dict(R=np.eye(3),t=np.array([1.,0,0]),inlier=.61,support=.53,rmse=.0105)
        _,ambiguity=rank_hypotheses([(a,.6),(b,.6)])
        self.assertLess(b['inlier']/a['inlier'],.9);self.assertGreater(ambiguity,.9)
    def test_vertical_evidence_ignores_ground(self):
        # 室外：错开 2 m 的解靠地面照样有九成内点；只数竖直面内点时它就不算竞争解（数字取自合成场景）
        a=dict(R=np.eye(3),t=np.zeros(3),inlier=.98,support=.97,inlier_v=.19,support_v=.18,rmse=.002)
        b=dict(R=np.eye(3),t=np.array([2.,0,0]),inlier=.91,support=.90,inlier_v=.12,support_v=.11,rmse=.002)
        self.assertGreater(rank_hypotheses([(a,.6),(b,.6)])[1],.9)
        self.assertLess(rank_hypotheses([(a,.6),(b,.6)],vertical=True)[1],.7)
    def test_empty_cloud_rejected(self):
        m=MapModel.__new__(MapModel);m.cfg=load();r=relocalize(m,np.zeros((0,3)),np.zeros(3));self.assertFalse(r['ok'])
    def test_single_plane_has_no_pose_observability(self):
        x,y=np.meshgrid(np.arange(-2,2,.08),np.arange(-2,2,.08));P=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
        m=MapModel.__new__(MapModel);m.cfg=load();m.nn=NNField(P)
        r=m.icp(P,[0,0,0,0]);self.assertLess(r['rank_ratio'],1e-6);self.assertLess(r['degen'],.12)
        m.zf=0.;self.assertFalse(relocalize(m,P,np.array([0.,0.,1.]))['ok'])


def outdoor_scene():
    """地面 z=-0.2（原点 = 起点时雷达，离地 0.2 m），三面墙、一根方柱、一面斜墙，外加 18 m 外一面应被裁掉的墙"""
    g=np.arange(-14,14,.1);x,y=np.meshgrid(g,g);parts=[np.c_[x.ravel(),y.ravel(),np.full(x.size,-.2)]]
    h=np.arange(-.2,2.8,.1)
    def wall(a,b):
        n=int(np.linalg.norm(np.subtract(b,a))/.1)+1;s=np.linspace(0,1,n)[:,None]
        xy=np.asarray(a)+s*(np.subtract(b,a));X,Z=np.repeat(xy,len(h),0),np.tile(h,n)
        return np.c_[X,Z]
    for a,b in [((-8,-8),(-8,8)),((-8,6),(10,6)),((4,-4),(5,-4)),((5,-4),(5,-5)),((5,-5),(4,-5)),((4,-5),(4,-4)),((5,-1),(10,-3)),((18,-6),(18,6))]:
        parts.append(wall(a,b))
    return np.concatenate(parts)


class OutdoorTests(unittest.TestCase):
    """室外配置：按原点裁图、候选只在起飞区、解在区外就拒绝"""
    @classmethod
    def setUpClass(cls):
        from drone_mapping import mapio
        cls.tmp=tempfile.TemporaryDirectory();d=cls.tmp.name;cls.P=outdoor_scene()
        mapio.write_pcd(os.path.join(d,'map_world.pcd'),cls.P.astype('f4'),np.zeros(len(cls.P),'f4'))
        with open(os.path.join(d,'meta.json'),'w') as f:json.dump(dict(floor=dict(height_m=.2,trusted=True)),f)
        cls.cfg=load(overrides=dict(crop_radius=12.,zone_radius=3.,max_range=9.,map_voxel=.1,ground_voxel=.2,split_ground=1))
        cls.M=MapModel(d,cls.cfg)
    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()
    def test_crop_and_candidates_stay_near_origin(self):
        self.assertLessEqual(np.hypot(self.M.P[:,0],self.M.P[:,1]).max(),12+1e-6)
        self.assertFalse((abs(self.M.P[:,0]-18)<.2).any())
        ii,jj=np.nonzero(self.M.cand);r=self.cfg['res']
        self.assertLessEqual(np.hypot(self.M.ox+(jj+.5)*r,self.M.oy+(ii+.5)*r).max(),3.5+1e-6)
    def look(self,p):
        # 实时点云：从 p 看出去的场景，drone_odom 原点就在传感器上（c=0）
        L=self.P-p;return L[np.linalg.norm(L,axis=1)<15]
    def test_inside_zone_localizes(self):
        p=np.array([1.,-.5,0.]);r=relocalize(self.M,self.look(p),np.zeros(3))
        self.assertTrue(r['ok'],r['why']);np.testing.assert_allclose(r['drone_map'],p,atol=.03)
    def test_outside_zone_rejected(self):
        r=relocalize(self.M,self.look(np.array([6.,1.,0.])),np.zeros(3));self.assertFalse(r['ok'])
    def test_zero_switches_and_zone_inside_crop(self):
        c=load();self.assertEqual((c['crop_radius'],c['zone_radius'],c['split_ground']),(0.,0.,0))
        with self.assertRaises(ValueError):load(overrides=dict(crop_radius=3.,zone_radius=3.))
        with self.assertRaises(ValueError):load(overrides=dict(fit_ok=0.))


if __name__=='__main__':unittest.main()
