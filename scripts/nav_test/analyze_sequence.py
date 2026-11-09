#!/usr/bin/env python3
"""Offline bag + goal_test4/test5 JSON metrics; no ROS master or FCU I/O.

Usage: python3 analyze_sequence.py flight.bag goal_test4_*.json --out analysis
Results are reference/odometry measurements; thrust requires the matching ULog.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np
import rosbag
from drone_navigation.core import Spline, wrap


def vec(v): return np.array([v.x, v.y, v.z], float)
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def analyze(bag_path, record):
    commands, odom, px, accepted, splines, faults = [], [], [], [], {}, []
    topics = ['/drone_nav/'+x for x in ('command','odom','executor_status','bspline','gateway_status')]
    topics += ['/mavros/local_position/odom']
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, m, stamp in bag.read_messages(topics=topics):
            t = m.header.stamp.to_sec() if hasattr(m, 'header') else stamp.to_sec()
            if topic.endswith('/command'):
                commands.append((t, vec(m.position), vec(m.velocity), vec(m.acceleration), m.yaw, m.terminal))
            elif topic.endswith('/odom'):
                row = (t, vec(m.pose.pose.position), yaw(m.pose.pose.orientation), np.linalg.norm(vec(m.twist.twist.linear)))
                (px if topic.startswith('/mavros') else odom).append(row)
            elif topic.endswith('/bspline'):
                splines[m.traj_id] = Spline(m.order,m.knots,[vec(p) for p in m.pos_pts],m.start_time.to_sec(),m.traj_id)
            else:
                s = json.loads(m.data)
                if s.get('state') in ('RUNNING','HOLD') and 'trajectory_id' in s:
                    accepted.append((s.get('stamp', t), s['trajectory_id']))
                if s.get('state') == 'FAULT' and s.get('reason') != 'task stopped; prepare before next route':
                    faults.append(dict(t=t, topic=topic, reason=s.get('reason','')))
    pt = np.array([o[0] for o in px]); pa = np.unwrap([o[2] for o in px])
    ot = np.array([o[0] for o in odom]); oa = np.unwrap([o[2] for o in odom])
    alignment = None
    if len(px)>1 and len(odom)>1:
        mask = (ot >= pt[0]) & (ot <= pt[-1]) & (ot < record['start_ros'])
        # The latest two seconds before the frozen route approximate the
        # gateway's initial static yaw offset; never infer absolute heading.
        if mask.any():
            mask &= ot >= ot[mask][-1]-2
            ds = np.interp(ot[mask], pt, pa)-oa[mask]
            alignment = math.atan2(np.sin(ds).mean(), np.cos(ds).mean())
    rows=[]
    for leg in record.get('legs', []):
        start, stop = leg['sent_ros'], leg.get('end_ros', record.get('start_ros',0))
        hold_start = leg.get('hold_start_ros', stop)
        hold_stop = hold_start+leg.get('hold_elapsed_s', 0)
        cs = [c for c in commands if start<=c[0]<=stop]
        obs = [o for o in odom if start<=o[0]<=hold_stop]
        ids=[]; switches=[]
        for t, ident in accepted:
            if start<=t<=stop and (not ids or ids[-1]!=ident):
                if ids and ident in splines and ids[-1] in splines:
                    old,new=splines[ids[-1]],splines[ident]
                    so,sn=old.state(t-old.start),new.state(t-new.start)
                    switches.append(dict(t=t,old_id=old.id,new_id=new.id,
                        **{'delta_'+k:float(np.linalg.norm(b-a)) for k,a,b in zip(('p_m','v_mps','a_mps2'),so,sn)}))
                ids.append(ident)
        goal=np.array(leg['goal_odom'])
        row=dict(index=leg['index'],direction=leg['direction'],end=leg['end'],
                 elapsed_s=leg.get('elapsed_s'),end_error_m=leg.get('end_error_m'),
                 end_height_error_m=leg.get('end_height_error_m'),
                 trajectory_ids=ids,replans=max(0,len(ids)-1),switches=switches,
                 command_v_peak_mps=max((float(np.linalg.norm(c[2])) for c in cs),default=None),
                 command_a_peak_mps2=max((float(np.linalg.norm(c[3])) for c in cs),default=None),
                 measured_v_peak_mps=max((float(o[3]) for o in obs),default=None))
        if leg.get('end_yaw_odom_deg') is not None and record.get('goal_yaw_odom_deg'):
            row['end_yaw_error_deg']=math.degrees(wrap(math.radians(leg['end_yaw_odom_deg']-
                record['goal_yaw_odom_deg'][leg['index']-1])))
        if cs:
            matched=[o for o in obs if cs[0][0]<=o[0]<=cs[-1][0]]
            if matched:
                ct=np.array([c[0] for c in cs]);cp=np.array([c[1] for c in cs])
                measured=np.array([o[1] for o in matched]);mt=np.array([o[0] for o in matched])
                reference=np.column_stack([np.interp(mt,ct,cp[:,j]) for j in range(3)])
                error=measured-reference
                row['tracking_error_peak_m']=float(np.linalg.norm(error,axis=1).max())
                row['height_tracking_rms_m']=float(np.sqrt(np.mean(error[:,2]**2)))
                row['height_tracking_peak_m']=float(np.abs(error[:,2]).max())
            jumps=[(float(np.linalg.norm(b[1]-a[1])),b[0]-a[0]) for a,b in zip(cs,cs[1:])]
            row['command_step_peak_m']=max((j for j,dt in jumps),default=0.)
            row['command_continuity_excess_max_m']=max((max(0.,j-.04-record['max_vel']*max(0.,dt)*1.2) for j,dt in jumps),default=0.)
            row['premature_terminal_samples']=int(sum(c[5] and np.linalg.norm(c[1]-goal)>.15 for c in cs))
            row['axis_v_peak_mps']=np.max(np.abs([c[2] for c in cs]),axis=0).tolist()
            row['intermediate_endpoints']=[splines[i].state(splines[i].duration)[0].tolist()
                for i in ids if i in splines and np.linalg.norm(splines[i].state(splines[i].duration)[0]-goal)>.02]
            turning=[cs[j][0] for j in range(1,len(cs)) if abs(wrap(cs[j][4]-cs[j-1][4]))>1e-5]
            row['turn_command_duration_s']=turning[-1]-turning[0] if len(turning)>1 else 0.
            row['switch_reference_jump_max_m']=max((s['delta_p_m'] for s in switches),default=0.)
        holds=[o for o in obs if o[0]>=hold_start]
        for label, samples in (('motion',obs),('hold',holds)):
            row[label+'_position_error_peak_m']=max((float(np.linalg.norm(o[1]-goal)) for o in samples),default=None)
        if alignment is not None:
            residuals=[abs(wrap(float(np.interp(o[0],pt,pa))-o[2]-alignment)) for o in obs if pt[0]<=o[0]<=pt[-1]]
            row['lio_px4_yaw_residual_peak_deg']=math.degrees(max(residuals)) if residuals else None
        rows.append(row)
    faults=[f for f in faults if record['start_ros']<=f['t']<=record.get('end_ros',float('inf'))]
    return dict(test=record['test'],bag=str(bag_path),end=record.get('end'),legs=rows,
                total_elapsed_s=(record['end_ros']-record['start_ros'] if 'end_ros' in record else None),
                yaw_alignment_rad=alignment, faults=faults,
                note='Ideal simulation or estimator evidence is not flight proof. Thrust needs matching ULog; null means missing coverage.')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag',type=Path);ap.add_argument('goal',type=Path);ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();record=json.loads(a.goal.read_text());result=analyze(a.bag,record)
    a.out.mkdir(parents=True,exist_ok=True)
    (a.out/'summary.json').write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False))
    rows=[{k:v for k,v in r.items() if not isinstance(v,(list,dict))} for r in result['legs']]
    if rows:
        with (a.out/'legs.csv').open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
            writer.writeheader();writer.writerows(rows)
    print(a.out/'summary.json')


if __name__=='__main__': main()
