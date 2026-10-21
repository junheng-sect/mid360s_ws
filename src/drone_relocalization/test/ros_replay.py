#!/usr/bin/env python3
"""Actual ROS messages against an isolated master and a synthetic stationary sensor stream.
No driver, FCU, rosbag play of vehicle topics, or hardware service is started.
"""
import json,os,signal,subprocess,threading,time,xmlrpc.client
from pathlib import Path
import numpy as np


def main():
    out=Path.home()/'test_data/reloc_acceptance/20261020';out.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:11531';os.environ['ROS_IP']='127.0.0.1';os.environ.pop('ROS_HOSTNAME',None)
    # Refuse to reuse ANY existing master on this port.
    import socket
    probe=socket.socket();probe.settimeout(.2)
    if probe.connect_ex(('127.0.0.1',11531))==0:raise RuntimeError('private test port already in use')
    probe.close();master_log=open(str(out/'ros_master.log'),'w')
    master=subprocess.Popen(['roscore','-p','11531'],stdout=master_log,stderr=subprocess.STDOUT,start_new_session=True)
    node=None;stop=threading.Event();thread=None;node_log=None
    def terminate(p):
        if p and p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=6)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3)
    try:
        for _ in range(100):
            try:
                if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/test')[0]==1:break
            except OSError:pass
            time.sleep(.1)
        else:raise RuntimeError('private master failed')
        import rospy,tf2_ros
        from rosgraph_msgs.msg import Clock
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String,Header
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs import point_cloud2
        from geometry_msgs.msg import PoseStamped,TransformStamped
        from tf2_msgs.msg import TFMessage
        from drone_relocalization.session import BagSession
        from drone_relocalization.geometry import quat_R,R_quat,unpack
        rospy.init_node('indoor_relocation_offline_test',disable_signals=True)
        S=BagSession(str(Path.home()/'test_data/mapping/flight_raw_000001.bag'))
        cloud=(S.cloud(1,2)-S.twc)@S.Rwc;row=S.pose(S.t0+2)
        frames=[point_cloud2.create_cloud_xyz32(Header(frame_id='camera_init'),p.astype('f4').tolist()) for p in np.array_split(cloud,20)]
        pub_od=rospy.Publisher('/Odometry',Odometry,queue_size=10);pub_cl=rospy.Publisher('/cloud_registered',PointCloud2,queue_size=10)
        broadcaster=tf2_ros.StaticTransformBroadcaster();tf=[]
        def init_tf(names):
            out=[]
            for (a,b),(R,t) in zip(names,[(S.Rwc,S.twc),(S.Rib,S.tib)]):
                tr=TransformStamped();tr.header.frame_id=a;tr.child_frame_id=b;tr.header.stamp=rospy.Time.now()
                tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=t
                tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w=R_quat(R);out.append(tr)
            return out
        tf=init_tf([('drone_odom','camera_init'),('body','drone_base')]);broadcaster.sendTransform(tf)
        # Pre-10-21 names, sent alone by a fresh broadcaster so the latched /tf_static holds only them.
        legacy_tf=init_tf([('world','camera_init'),('body','base_link')])
        pub_clock=rospy.Publisher('/clock',Clock,queue_size=1)
        state=dict(status=None,result=None,poses=[],tfs=[],maps=0,enabled=True,cloud_enabled=True,rewind=False)
        def status(m):state['status']=json.loads(m.data)
        def result(m):
            r=json.loads(m.data)
            if r.get('confirmed'):state['result']=r
        def pose(m):state['poses'].append(m)
        def on_tf(m):state['tfs'].extend(t for t in m.transforms if t.header.frame_id=='drone_map' and t.child_frame_id=='drone_odom')
        def on_map(m):
            if m.header.frame_id=='drone_map' and m.width>1000:state['maps']+=1
        subs=[rospy.Subscriber('/drone_reloc/status',String,status),rospy.Subscriber('/drone_reloc/result',String,result),
              rospy.Subscriber('/drone_reloc/pose',PoseStamped,pose),rospy.Subscriber('/tf',TFMessage,on_tf),rospy.Subscriber('/drone_reloc/map',PointCloud2,on_map)]
        def publish():
            n=0
            while not stop.is_set():
                if state['enabled']:
                    stamp=rospy.Time.now()-rospy.Duration(5 if state['rewind'] else 0)
                    pub_clock.publish(Clock(clock=stamp))
                    od=Odometry();od.header.stamp=stamp;od.header.frame_id='camera_init';od.child_frame_id='body'
                    od.pose.pose.position.x,od.pose.pose.position.y,od.pose.pose.position.z=row[1:4]
                    od.pose.pose.orientation.x,od.pose.pose.orientation.y,od.pose.pose.orientation.z,od.pose.pose.orientation.w=row[4:8]
                    cl=frames[n%len(frames)];cl.header.stamp=stamp;pub_od.publish(od)
                    if state['cloud_enabled']:pub_cl.publish(cl)
                    n+=1
                stop.wait(.1)
        thread=threading.Thread(target=publish,daemon=True);thread.start();results=[]
        for fault in ['stale','cloud_stale','rewind','tf_change','clock_freeze','legacy_names']:
            state.update(status=None,result=None,poses=[],tfs=[],maps=0,enabled=True,cloud_enabled=True,rewind=False)
            rospy.set_param('/use_sim_time',fault=='clock_freeze')
            if fault=='legacy_names':tf2_ros.StaticTransformBroadcaster().sendTransform(legacy_tf)
            else:broadcaster.sendTransform(tf)
            node_log=open(str(out/('ros_'+fault+'.log')),'w')
            node=subprocess.Popen(['rosrun','drone_relocalization','relocalize','--map',str(Path.home()/'test_data/mapping/20261018_194304_voxelslam'),'--save',''],stdout=node_log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+65
            while time.monotonic()<deadline and not (state['status'] and state['status']['valid'] and state['poses'] and state['result']):
                if node.poll() is not None:raise AssertionError('node exited: '+str(out/('ros_'+fault+'.log')))
                time.sleep(.1)
            assert state['status'] and state['status']['valid'],'no LOCALIZED for '+fault
            assert state['status']['confirmations']>=2 and state['maps']>0
            time.sleep(.5);r=state['result'];R,t=unpack(r);p=state['poses'][-1];q=p.pose.orientation
            actual=np.array([p.pose.position.x,p.pose.position.y,p.pose.position.z])
            expected=R@(S.Rwc@(row[1:4]+quat_R(row[4:8])@S.tib)+S.twc)+t
            expected_R=R@S.Rwc@quat_R(row[4:8])@S.Rib
            assert np.linalg.norm(actual-expected)<1e-6
            np.testing.assert_allclose(quat_R([q.x,q.y,q.z,q.w]),expected_R,atol=1e-6)
            assert state['tfs'] and p.header.frame_id=='drone_map'
            if fault in ('stale','clock_freeze','legacy_names'):state['enabled']=False
            elif fault=='cloud_stale':state['cloud_enabled']=False
            elif fault=='rewind':state['rewind']=True
            else:
                changed=TransformStamped();changed.header.frame_id='drone_odom';changed.child_frame_id='camera_init';changed.header.stamp=rospy.Time.now()
                changed.transform.translation.x=.01
                changed.transform.rotation.x,changed.transform.rotation.y,changed.transform.rotation.z,changed.transform.rotation.w=R_quat(S.Rwc)
                broadcaster.sendTransform(changed)
            deadline=time.monotonic()+2
            while time.monotonic()<deadline and state['status']['valid']:time.sleep(.05)
            assert not state['status']['valid'] and state['status']['state']=='STALE',state['status']
            time.sleep(.3);counts=(len(state['poses']),len(state['tfs']));time.sleep(.7)
            assert counts==(len(state['poses']),len(state['tfs'])),'continued publishing after invalidation'
            results.append(dict(fault=fault,passed=True,status=state['status'],pose_count=counts[0],tf_count=counts[1]))
            print('ROS PASS',fault,flush=True);terminate(node);node=None;node_log.close();node_log=None
        S.close();json.dump(dict(master_port=11531,hardware=False,results=results),open(str(out/'ros_replay.json'),'w'),ensure_ascii=False,indent=2)
    finally:
        stop.set()
        if thread:thread.join(1)
        terminate(node);terminate(master)
        if node_log:node_log.close()
        master_log.close()


if __name__=='__main__':main()
