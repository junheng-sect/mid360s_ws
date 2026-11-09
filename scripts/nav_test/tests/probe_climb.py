#!/usr/bin/env python3
"""Full-flight replay: query hypothetical test5 paths against recorded observations.

Replay recorded /drone_nav/{odom,sensor_odom,cloud} and /mavros/state with --clock.
This does not invent observations at an unflown height and does not send goals.
"""
import collections
import json
import math
import sys
from pathlib import Path
import numpy as np
import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from traj_utils.srv import CheckTrajectory
from drone_navigation.ros_common import path_of

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from nav_sequence import sequence_points

state=dict(odom=None,off=False,since=None,anchor=None,run=0)
rows=[];anchors=[]

def fcu(m):
    off=m.connected and m.armed and m.mode=='OFFBOARD'
    if off and not state['off']:
        state.update(since=rospy.Time.now().to_sec(),anchor=None,run=state['run']+1)
    state['off']=off

def tick(_):
    now=rospy.Time.now().to_sec();o=state['odom']
    if not state['off'] or state['since'] is None or now-state['since']<3 or o is None:return
    if state['anchor'] is None:
        p=o.pose.pose.position;q=o.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        state['anchor']=np.array([p.x,p.y,p.z]),yaw
        anchors.append(dict(run=state['run'],t=now,p=state['anchor'][0].tolist(),yaw=yaw))
    p,yaw=state['anchor'];goals,_,names=sequence_points('test5',p,yaw)
    for i,g in enumerate(goals[:4]):
        try:
            r=check(path_of([p if i==0 else goals[i-1],g]));why=r.reason
            category=('known_free' if r.valid else 'unknown' if 'unknown' in why else
                      'occupied' if 'occupied' in why else 'outside' if 'outside' in why else 'unavailable')
            rows.append(dict(t=now,run=state['run'],leg=i+1,name=names[i],valid=r.valid,category=category,reason=why))
        except Exception as e:
            rows.append(dict(t=now,run=state['run'],leg=i+1,valid=False,category='unavailable',reason=str(e)))

def save():
    data=dict(anchors=anchors,counts=dict(collections.Counter(r['category'] for r in rows)),rows=rows,
              note='Recorded map observations only. A pass is historical coverage, not proof of observations during a future climb. Unknown coverage blocks the live segment check.')
    Path(sys.argv[1]).write_text(json.dumps(data,indent=2,ensure_ascii=False))

if __name__=='__main__':
    rospy.init_node('test5_climb_replay_probe')
    rospy.Subscriber('/drone_nav/odom',Odometry,lambda m:state.update(odom=m),queue_size=1)
    rospy.Subscriber('/mavros/state',State,fcu,queue_size=5)
    rospy.wait_for_service('/drone_nav/check_trajectory')
    check=rospy.ServiceProxy('/drone_nav/check_trajectory',CheckTrajectory)
    rospy.on_shutdown(save);rospy.Timer(rospy.Duration(.5),tick);rospy.spin()
