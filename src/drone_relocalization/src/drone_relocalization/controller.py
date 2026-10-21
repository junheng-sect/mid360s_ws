"""Thread-safe callers serialize these operations. Clock is monotonic receipt time, never ROS time."""
from collections import deque
import math
import numpy as np
from .geometry import quat_R,angle,distance
from .settings import load


class StartupController:
    def __init__(self,cfg=None,now=0.):
        self.cfg=cfg or load();self.started=now;self.state='WAITING';self.why='等待新鲜里程计和点云'
        self.epoch=0;self.history=deque();self.last_odom=None;self.last_cloud=None
        self.recv_odom=None;self.recv_cloud=None;self.pending=None;self.result=None;self.confirmed=0;self.attempts=0
        self.anchor=None;self.start_stamp=None;self.start_time=None;self.end_stamp=None;self.cloud_count=0;self.trace=[]

    @property
    def valid(self):return self.state=='LOCALIZED' and self.result is not None

    @property
    def terminal(self):return self.state in ('FAILED','STALE')

    def set_state(self,state,why=''):
        if (state,why)!=(self.state,self.why):self.trace.append(dict(state=state,why=why,attempts=self.attempts))
        self.state,self.why=state,why

    def invalidate(self,why,terminal=False):
        was_locked=self.state=='LOCALIZED';self.epoch+=1;self.pending=None;self.result=None;self.confirmed=0
        self.anchor=None;self.start_stamp=None;self.history.clear()
        self.set_state('STALE' if was_locked else ('FAILED' if terminal else 'WAITING'),why)

    def observe_odom(self,row,now):
        if self.terminal:return
        row=np.asarray(row,float)
        try:
            if row.shape!=(8,) or not np.isfinite(row).all():raise ValueError()
            R=quat_R(row[4:8])
        except ValueError:
            self.invalidate('里程计格式或四元数非法',True);return
        if self.last_odom is not None and row[0]<=self.last_odom[0]:
            self.invalidate('里程计时间回退或重复，需重启',True);return
        self.last_odom=row.copy();self.recv_odom=now;self.history.append(row.copy())
        while self.history and row[0]-self.history[0][0]>1.3:self.history.popleft()
        if self.anchor is not None and self.state!='LOCALIZED':
            if np.linalg.norm(row[1:4]-self.anchor[1:4])>=self.cfg['still_m'] or math.degrees(angle(R@quat_R(self.anchor[4:8]).T))>=self.cfg['still_deg']:
                self.invalidate('采集或确认期间发生运动，重新等待静止')

    def observe_cloud(self,stamp,now):
        if self.terminal:return
        if not math.isfinite(stamp) or (self.last_cloud is not None and stamp<=self.last_cloud):
            self.invalidate('点云时间回退或重复，需重启',True);return
        self.last_cloud=stamp;self.recv_cloud=now
        if self.state=='COLLECTING' and stamp>=self.start_stamp:
            self.end_stamp=stamp;self.cloud_count+=1

    def fresh(self,now,cloud=True):
        return (self.recv_odom is not None and now-self.recv_odom<=self.cfg['freshness'] and
                (not cloud or (self.recv_cloud is not None and now-self.recv_cloud<=self.cfg['freshness'])))

    def tick(self,now):
        if self.terminal:return
        if self.valid:
            if not self.fresh(now):self.invalidate('里程计或点云断流，旧定位失效，需重启',True)
            return
        if now-self.started>=self.cfg['timeout']:
            self.invalidate('启动定位超过总等待上限',True);return
        if self.recv_odom is not None and not self.fresh(now):
            if self.state!='WAITING' or self.pending is not None:self.invalidate('输入断流，清除未确认候选')
            return
        if self.state=='WAITING' and self.fresh(now) and len(self.history)>=10:
            h=np.array(self.history);h=h[h[:,0]>=h[-1,0]-1.05]
            if h[-1,0]-h[0,0]<.9:return
            a=h[0]
            if np.linalg.norm(h[:,1:4]-a[1:4],axis=1).max()>=self.cfg['still_m']:return
            if max(math.degrees(angle(quat_R(q)@quat_R(a[4:8]).T)) for q in h[:,4:8])>=self.cfg['still_deg']:return
            if self.attempts>=self.cfg['tries']:
                self.invalidate('已达到最大搜索次数',True);return
            if self.anchor is None:self.anchor=h[-1].copy()
            self.start_stamp=float(h[-1,0]);self.start_time=now;self.end_stamp=None;self.cloud_count=0
            self.set_state('COLLECTING','采集独立静止窗口')

    def job(self,now):
        self.tick(now)
        if self.state!='COLLECTING' or self.end_stamp is None:return None
        if self.end_stamp-self.start_stamp<self.cfg['window']-1e-6 or self.cloud_count<10:return None
        if not self.fresh(now):return None
        self.attempts+=1;self.set_state('SEARCHING','全局匹配中')
        return dict(token=self.epoch,start=self.start_stamp,end=self.start_stamp+self.cfg['window'],
                    center=self.last_odom[1:4].copy(),hint=self.pending)

    def submit(self,result,token,now):
        self.tick(now)
        if token!=self.epoch or self.state!='SEARCHING':return False
        if not self.fresh(now):self.invalidate('匹配完成时输入已断流');return False
        if not result.get('ok'):
            self.pending=None;self.confirmed=0
            self.set_state('WAITING',result.get('why','匹配失败'))
        else:
            consistent=False
            if self.pending is not None:
                d,a=distance(self.pending,result);consistent=d<=self.cfg['consensus_m'] and a<=self.cfg['consensus_deg']
            self.confirmed=self.confirmed+1 if consistent else 1;self.pending=result
            if self.confirmed>=self.cfg['confirmations']:
                self.result=result;self.set_state('LOCALIZED','启动解已确认；之后仅由里程计传播，不持续匹配地图');return True
            self.set_state('WAITING','等待第二个独立窗口确认' if self.confirmed==1 else '等待独立窗口确认')
        if self.attempts>=self.cfg['tries']:self.invalidate('搜索次数耗尽，未取得连续一致解',True)
        return False

    def status(self,stamp):
        return dict(schema_version=2,stamp=float(stamp),state=self.state,valid=self.valid,why=self.why,
                    attempts=self.attempts,confirmations=self.confirmed,tracking=False)
