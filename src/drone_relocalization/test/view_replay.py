#!/usr/bin/env python3
"""RViz comparison view against an isolated master: identity before localization, aligned after, identity again
once the localization node dies. A stationary stream is built from one window of the map's own raw.bag and moved
by a known rigid offset P in drone_odom, as if FAST-LIO had started away from the map origin.
No driver, FCU or hardware is started. --demo binds the master to the Wi-Fi address and holds each phase longer
so the local Docker RViz can watch it: ./scripts/run_rviz.sh --reloc --ros-master http://192.168.31.32:11531
"""
import argparse,json,math,os,signal,subprocess,threading,time,xmlrpc.client
from pathlib import Path
import numpy as np

MAP=Path.home()/'test_data/mapping/20261021_110311_indoor'
PORT=11531
SAVE=Path.home()/'test_data/reloc'  # relocalize.launch keeps the node's default --save
P_YAW,P_T=math.radians(60.),np.array([1.2,-.6,0.])


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--demo',action='store_true')
    ap.add_argument('--out',default=str(Path.home()/'test_data/reloc_acceptance/20261021'));a=ap.parse_args()
    out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
    host='192.168.31.32' if a.demo else '127.0.0.1'
    os.environ['ROS_MASTER_URI']='http://%s:%d'%(host,PORT);os.environ['ROS_IP']=host;os.environ.pop('ROS_HOSTNAME',None)
    import socket
    probe=socket.socket();probe.settimeout(.2)
    if probe.connect_ex(('127.0.0.1',PORT))==0:raise RuntimeError('private test port already in use')
    probe.close();master_log=open(str(out/'view_master.log'),'w')
    master=subprocess.Popen(['roscore','-p',str(PORT)],stdout=master_log,stderr=subprocess.STDOUT,start_new_session=True)
    launch=None;launch_log=None;stop=threading.Event();thread=None;saved=set(SAVE.glob('drone_reloc_*.json'));had_save=SAVE.exists()
    def terminate(p):
        if p and p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3)
    try:
        api=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        for _ in range(100):
            try:
                if api.getPid('/test')[0]==1:break
            except OSError:pass
            time.sleep(.1)
        else:raise RuntimeError('private master failed')
        import rospy,tf2_ros
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String,Header
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs import point_cloud2
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage
        from visualization_msgs.msg import Marker
        from drone_relocalization.session import BagSession
        from drone_relocalization.matching import MapModel
        from drone_relocalization.geometry import quat_R,R_quat,rotz,unpack,angle
        rospy.init_node('reloc_view_test',disable_signals=True)
        S=BagSession(str(MAP/'raw.bag'));M=MapModel(str(MAP))
        Rp=rotz(P_YAW)
        cw=S.cloud(1,2)@Rp.T+P_T                         # perturbed drone_odom cloud
        cloud=(cw-S.twc)@S.Rwc                           # back to camera_init
        row=S.pose(S.t0+2);Rb=S.Rwc@quat_R(row[4:8]);pb=S.Rwc@row[1:4]+S.twc
        Rb_ci=S.Rwc.T@Rp@Rb;pb_ci=S.Rwc.T@(Rp@pb+P_T-S.twc)
        frames=[point_cloud2.create_cloud_xyz32(Header(frame_id='camera_init'),p.astype('f4').tolist()) for p in np.array_split(cloud,20)]
        pub_od=rospy.Publisher('/Odometry',Odometry,queue_size=10);pub_cl=rospy.Publisher('/cloud_registered',PointCloud2,queue_size=10)
        static=tf2_ros.StaticTransformBroadcaster();dyn=tf2_ros.TransformBroadcaster();tf=[]
        for p_,c_,R,t in [('drone_odom','camera_init',S.Rwc,S.twc),('body','drone_base',S.Rib,S.tib)]:
            tr=TransformStamped();tr.header.frame_id=p_;tr.child_frame_id=c_;tr.header.stamp=rospy.Time.now()
            tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=t
            tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w=R_quat(R);tf.append(tr)
        static.sendTransform(tf)
        state=dict(status=None,result=None,view_tf=None,view_map=None,map=None,text=None,jiggle=True)
        def on_status(m):state['status']=json.loads(m.data)
        def on_result(m):
            r=json.loads(m.data)
            if r.get('confirmed'):state['result']=r
        def on_static(m):
            for t in m.transforms:
                if t.header.frame_id=='drone_odom' and t.child_frame_id=='drone_map_view':
                    q,p=t.transform.rotation,t.transform.translation
                    state['view_tf']=(quat_R([q.x,q.y,q.z,q.w]),np.array([p.x,p.y,p.z]))
        def on_view_map(m):state['view_map']=(m.header.frame_id,m.width*m.height)
        def on_map(m):state['map']=(m.header.frame_id,m.width*m.height)
        def on_text(m):state['text']=m.text
        state['subs']=[rospy.Subscriber('/drone_reloc/status',String,on_status),rospy.Subscriber('/drone_reloc/result',String,on_result),
              rospy.Subscriber('/tf_static',TFMessage,on_static),rospy.Subscriber('/drone_reloc/view/map',PointCloud2,on_view_map),
              rospy.Subscriber('/drone_reloc/map',PointCloud2,on_map),rospy.Subscriber('/drone_reloc/view/status_text',Marker,on_text)]
        def publish():
            n=0
            while not stop.is_set():
                stamp=rospy.Time.now()
                # Hand-held wobble keeps the start state machine waiting, so the misaligned view stays visible.
                d=np.array([.05*math.sin(n*.9),.05*math.cos(n*.7),0.]) if state['jiggle'] else np.zeros(3)
                p=pb_ci+d;q=R_quat(Rb_ci)
                od=Odometry();od.header.stamp=stamp;od.header.frame_id='camera_init';od.child_frame_id='body'
                od.pose.pose.position.x,od.pose.pose.position.y,od.pose.pose.position.z=p
                od.pose.pose.orientation.x,od.pose.pose.orientation.y,od.pose.pose.orientation.z,od.pose.pose.orientation.w=q
                tr=TransformStamped();tr.header.stamp=stamp;tr.header.frame_id='camera_init';tr.child_frame_id='body'
                tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=p
                tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w=q
                cl=frames[n%len(frames)];cl.header.stamp=stamp;pub_od.publish(od);dyn.sendTransform(tr);pub_cl.publish(cl)
                n+=1;stop.wait(.1)
        thread=threading.Thread(target=publish,daemon=True);thread.start()
        def wait(cond,secs,what):
            deadline=time.monotonic()+secs
            while time.monotonic()<deadline:
                if launch.poll() is not None:raise AssertionError('roslaunch exited: '+str(out/'view_launch.log'))
                if cond():return
                time.sleep(.1)
            raise AssertionError('timeout: '+what)
        def fit():
            # Live cloud drawn in drone_map_view through the view TF, scored against the map's own NN field.
            # Wall band only: a yaw/xy offset leaves floor and ceiling points on their planes.
            R,t=state['view_tf'];pv=((cloud@S.Rwc.T+S.twc)-t)@R;pv=pv[(pv[:,2]>M.zf+.3)&(pv[:,2]<M.zf+2.)]
            r,_=M.nn.residual(pv,.15);inl=np.isfinite(r)&(abs(r)<.05)
            return dict(inlier=float(inl.mean()),median_abs_m=float(np.median(abs(r[inl]))) if inl.any() else None)
        launch_log=open(str(out/'view_launch.log'),'w')
        launch=subprocess.Popen(['roslaunch','drone_relocalization','relocalize.launch','map_dir:='+str(MAP)],
                                stdout=launch_log,stderr=subprocess.STDOUT,start_new_session=True)
        report=dict(master='%s:%d'%(host,PORT),hardware=False,perturb=dict(yaw_deg=math.degrees(P_YAW),t=P_T.tolist()))
        # 1. before localization: identity view, scan visibly off the map
        wait(lambda:state['view_tf'] is not None and state['view_map'] and state['map'] and state['status'],40,'view TF/map/status')
        time.sleep(25 if a.demo else 4)
        R,t=state['view_tf'];assert not state['status']['valid'],state['status']
        assert np.allclose(R,np.eye(3)) and np.allclose(t,0),'view not identity before localization'
        assert state['view_map']==('drone_map_view',state['map'][1]) and state['map'][0]=='drone_map',(state['view_map'],state['map'])
        assert state['text'] and 'not localized' in state['text'],state['text']
        before=fit();report.update(before=before,text_before=state['text'])
        assert before['inlier']<.3,before
        print('PASS before: identity view, inlier %.3f'%before['inlier'],flush=True)
        # 2. hold still: localized view equals the inverse of drone_map<-drone_odom and the scan lands on the map
        state['jiggle']=False
        wait(lambda:state['status']['valid'] and state['result'],70,'LOCALIZED')
        time.sleep(1.)
        Rm,tm=unpack(state['result']);R,t=state['view_tf']
        np.testing.assert_allclose(R,Rm.T,atol=1e-6);np.testing.assert_allclose(t,-Rm.T@tm,atol=1e-6)
        E_R=Rm@Rp;E_t=Rm@P_T+tm                          # map<-world_orig, should be close to identity
        after=fit();report.update(after=after,text_after=state['text'],residual_vs_P=dict(t_m=float(np.linalg.norm(E_t)),deg=math.degrees(angle(E_R))))
        assert after['inlier']>.6 and after['median_abs_m']<.02,after  # NN field only scores planar cells
        assert np.linalg.norm(E_t)<.1 and math.degrees(angle(E_R))<1.,report['residual_vs_P']
        assert state['text']=='LOCALIZED - scan aligned to map',state['text']
        # The localization node announces the position in its terminal once confirmed.
        launch_log.flush();lines=[s for s in open(str(out/'view_launch.log'),encoding='utf-8') if '定位成功' in s]
        assert lines,'no position line in '+str(out/'view_launch.log');report['position_line']=lines[-1].strip()
        pubs={k:v for k,v in api.getSystemState('/test')[2][0]}
        assert '/reloc_view' not in pubs.get('/tf',[]),pubs.get('/tf')
        print('PASS after: inlier %.3f median %.4f m; T*P off by %.3f m / %.2f deg'%(after['inlier'],after['median_abs_m'],
              report['residual_vs_P']['t_m'],report['residual_vs_P']['deg']),flush=True)
        if a.demo:time.sleep(60)
        # 3. kill -9 the localization node (no shutdown notice): status goes silent, view falls back to identity
        code,_,uri=api.lookupNode('/test','/drone_reloc');assert code==1,uri
        pid=xmlrpc.client.ServerProxy(uri).getPid('/test')[2];os.kill(pid,signal.SIGKILL);killed=time.monotonic()
        def reverted():
            R,t=state['view_tf'];return np.allclose(R,np.eye(3)) and np.allclose(t,0)
        deadline=killed+3.5
        while time.monotonic()<deadline and not reverted():time.sleep(.05)
        assert reverted(),'view did not revert after the node died';report['revert_s']=time.monotonic()-killed
        time.sleep(.5);assert state['text']=='NO /drone_reloc/status - not localized',state['text']
        print('PASS revert: identity %.2fs after kill -9'%report['revert_s'],flush=True)
        if a.demo:time.sleep(10)
        S.close();json.dump(report,open(str(out/'view_replay.json'),'w'),ensure_ascii=False,indent=2)
        print(json.dumps(report,ensure_ascii=False),flush=True)
    finally:
        stop.set()
        if thread:thread.join(1)
        terminate(launch);terminate(master)
        # Synthetic results must not sit next to real ones.
        for f in set(SAVE.glob('drone_reloc_*.json'))-saved:f.unlink()
        if not had_save and SAVE.is_dir() and not any(SAVE.iterdir()):SAVE.rmdir()
        if launch_log:launch_log.close()
        master_log.close()


if __name__=='__main__':main()
