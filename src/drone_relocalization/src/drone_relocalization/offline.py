"""Offline startup simulator: advance recorded inputs during the actual solver elapsed time."""
import time
import numpy as np
from .controller import StartupController
from .matching import relocalize


def startup(M,S,at=0.):
    cfg=M.cfg;ctl=StartupController(cfg,now=0.);lidar=S.lidar_stamps();events=[]
    for i,row in enumerate(S.od):
        if row[0]>=S.t0+at:events.append((row[0]-S.t0-at,0,i))
    for i,stamp in enumerate(lidar):
        if stamp>=S.t0+at:events.append((stamp-S.t0-at,1,i))
    events.sort();pos=0;now=0.;attempt_results=[]
    def advance(end):
        nonlocal pos,now
        # Deadline/freshness are checked at 50 ms intervals, even across a missing-data interval.
        while now<end-1e-9 and not ctl.terminal:
            until=min(end,now+.05)
            while pos<len(events) and events[pos][0]<=until+1e-9:
                tm,kind,i=events[pos];pos+=1
                if kind==0:
                    row=S.od[i].copy();row[1:4]=S.Rwc@row[1:4]+S.twc
                    from .geometry import quat_R,R_quat
                    row[4:8]=R_quat(S.Rwc@quat_R(row[4:8]));ctl.observe_odom(row,tm)
                else:ctl.observe_cloud(float(lidar[i]),tm)
            now=until;ctl.tick(now)
    while now<min(cfg['timeout']+.1,S.dur-at+.6) and not ctl.terminal and not ctl.valid:
        advance(now+.05);job=ctl.job(now)
        if job:
            tic=time.monotonic();cloud=S.cloud(job['start']-S.t0,cfg['window'])
            center,_=S.pose_at((job['start']+job['end'])/2)
            result=relocalize(M,cloud,center,hint=job.get('hint'));elapsed=time.monotonic()-tic
            advance(now+elapsed);accepted=ctl.submit(result,job['token'],now)
            attempt_results.append(dict(start=job['start']-S.t0,end=job['end']-S.t0,elapsed_s=elapsed,accepted=accepted,result=result))
    if not ctl.valid and not ctl.terminal:ctl.invalidate('包结束前未取得启动定位',True)
    return dict(schema_version=2,ok=ctl.valid,why=ctl.why,elapsed_data_s=now,status=ctl.status(S.t0+at+now),
                result=ctl.result,attempts=attempt_results,trace=ctl.trace)
