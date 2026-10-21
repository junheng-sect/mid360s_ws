"""全局2D候选 + 六自由度点到面匹配。保留独立空间峰，失败不输出有效变换。"""
import hashlib
import json
import math
import os
import tempfile
import time
import zipfile
import cv2
import numpy as np
from drone_mapping import mapio as bm
from .geometry import exp, rotz, yaw, wrap, transform, angle
from .settings import load

NN_REACH=.15
CACHE_VERSION='indoor-se3-v2-1'

class NNField:
    """地图最近点查找表：0.05 m 栅格，地图点 NN_REACH 以内的每个格子存离格子中心最近的地图点；再给每个地图点
    用 10 个近邻做 PCA 算法向。ICP 里查询点 → 所在格子 → 最近地图点和法向 → 点到面残差。
    （一开始用 0.2 m 体素拟合的平面做对应，墙角、杂物处平面不准，自测会被带偏 2–7 cm、0.3°）"""

    def __init__(self, P, res=0.05, reach=NN_REACH):
        t0 = time.time()
        self.P, self.res = P, res
        R = int(math.ceil(reach / res))
        self.lo = P.min(axis=0) - (R + 1) * res
        self.dims = (np.floor((P.max(axis=0) - self.lo) / res).astype(np.int64) + R + 2)
        base = np.floor((P - self.lo) / res).astype(np.int64)
        offs = [(i, j, k) for i in range(-R, R + 1) for j in range(-R, R + 1) for k in range(-R, R + 1)
                if i * i + j * j + k * k <= (R + 0.5) ** 2]
        lin, d2, idx = [], [], []
        ar = np.arange(len(P))
        for o in offs:
            c = base + o
            ctr = self.lo + (c + 0.5) * res
            lin.append(np.ravel_multi_index(c.T, self.dims))
            d2.append(((ctr - P) ** 2).sum(axis=1).astype(np.float32))
            idx.append(ar)
        lin, d2, idx = np.concatenate(lin), np.concatenate(d2), np.concatenate(idx)
        o = np.lexsort((d2, lin))
        lin, idx = lin[o], idx[o]
        first = np.r_[True, lin[1:] != lin[:-1]]
        self.grid = np.full(int(np.prod(self.dims)), -1, np.int32)
        self.grid[lin[first]] = idx[first]
        # 法向：10 近邻 PCA；不够平的点（λ0 / Σλ ≥ 0.1）不当对应
        cv2.setRNGSeed(0)
        fl = cv2.flann_Index(P.astype(np.float32), dict(algorithm=1, trees=4))
        nb, _ = fl.knnSearch(P.astype(np.float32), 10, params=dict(checks=64))
        X = P[nb] - P[nb].mean(axis=1, keepdims=True)
        w, v = np.linalg.eigh(np.einsum('nki,nkj->nij', X, X))
        self.n = v[:, :, 0]
        self.good = w[:, 0] / np.maximum(w.sum(axis=1), 1e-12) < 0.1
        self.build_s = time.time() - t0

    def save(self,path,key):
        parent=os.path.dirname(path)
        fd,tmp=tempfile.mkstemp(prefix='.reloc-v2-',suffix='.npz',dir=parent)
        os.close(fd)
        try:
            np.savez(tmp,key=key,lo=self.lo,dims=self.dims,grid=self.grid,n=self.n,good=self.good,res=self.res)
            os.replace(tmp,path)
        finally:
            if os.path.exists(tmp):os.unlink(tmp)

    @classmethod
    def load(cls,path,P,key):
        try:
            if not zipfile.is_zipfile(path):return None
            with np.load(path,allow_pickle=False) as z:
                if str(z['key'])!=key:return None
                lo,dims,grid,n,good,res=(z[k] for k in ('lo','dims','grid','n','good','res'))
                if lo.shape!=(3,) or dims.shape!=(3,) or dims.dtype.kind not in 'iu' or (dims<=0).any():return None
                if grid.shape!=(int(np.prod(dims)),) or grid.dtype!=np.int32:return None
                if n.shape!=P.shape or good.shape!=(len(P),) or good.dtype!=np.bool_:return None
                if not np.isfinite(lo).all() or not np.isfinite(n).all() or float(res)<=0:return None
                if grid.min() < -1 or grid.max()>=len(P):return None
                self=cls.__new__(cls);self.P=P;self.res=float(res);self.lo=lo;self.dims=dims
                self.grid=grid;self.n=n;self.good=good;self.build_s=0.
                return self
        except (OSError,ValueError,KeyError,EOFError,TypeError,OverflowError,zipfile.BadZipFile):return None

    def residual(self, Q, dmax):
        """返回 (点到面残差，没有对应 / 最近点远于 dmax / 不平的为 inf；对应法向)"""
        c = np.floor((Q - self.lo) / self.res).astype(np.int64)
        ok = ((c >= 0) & (c < self.dims)).all(axis=1)
        i = np.full(len(Q), -1, np.int64)
        i[ok] = self.grid[np.ravel_multi_index(c[ok].T, self.dims)]
        ok = i >= 0
        ok[ok] = self.good[i[ok]]
        m = self.P[np.maximum(i, 0)]
        ok &= ((Q - m) ** 2).sum(axis=1) < dmax * dmax
        r = np.full(len(Q), np.inf)
        n = self.n[np.maximum(i, 0)]
        r[ok] = np.einsum('ij,ij->i', Q[ok] - m[ok], n[ok])
        return r, n


class MapModel:
    def __init__(self,d,cfg=None):
        tic=time.monotonic();self.cfg=cfg or load();c=self.cfg
        d=os.path.abspath(os.path.expanduser(d));pcd=os.path.join(d,'map_world.pcd')
        with open(os.path.join(d,'meta.json'),'rb') as f:meta_bytes=f.read()
        meta=json.loads(meta_bytes);P,_=bm.read_pcd(pcd)
        if len(P)<1000:raise ValueError('地图点太少')
        floor=meta.get('floor') or bm.floor_fit(P)
        if not floor or not floor.get('trusted',False):raise ValueError('地图没有可信的地面记录')
        self.zf=-float(floor['height_m'])
        lo,hi=np.percentile(P,.5,axis=0)-2,np.percentile(P,99.5,axis=0)+2
        P=P[((P>=lo)&(P<=hi)).all(1)].astype(float);self.P=P
        dims=np.ceil((np.ptp(P,axis=0)+2*c['nn_reach']+.2)/c['res']).astype('i8')
        rad=int(math.ceil(c['nn_reach']/c['res']))
        neighbors=sum(i*i+j*j+k*k<=(rad+.5)**2 for i in range(-rad,rad+1) for j in range(-rad,rad+1) for k in range(-rad,rad+1))
        if int(np.prod(dims))>c['max_grid_cells'] or len(P)*neighbors*32>c['max_expand_bytes']:
            raise ValueError('地图超出室内索引内存预算，请裁分地图；未分配大数组')
        digest=hashlib.sha256();digest.update(meta_bytes);digest.update(CACHE_VERSION.encode())
        digest.update(json.dumps(c,sort_keys=True).encode())
        with open(pcd,'rb') as f:
            for chunk in iter(lambda:f.read(1048576),b''):digest.update(chunk)
        self.key=digest.hexdigest();cache=os.path.join(d,'reloc_cache_v2.npz')
        self.nn=NNField.load(cache,P,self.key) if os.path.isfile(cache) else None
        self.cache_hit=self.nn is not None
        if self.nn is None:
            self.nn=NNField(P,res=c['res'],reach=c['nn_reach'])
            try:self.nn.save(cache,self.key)
            except OSError:pass  # read-only map remains usable
        wall=P[(P[:,2]>self.zf+.5)&(P[:,2]<self.zf+2.)]
        ground=P[abs(P[:,2]-self.zf)<.08]
        self.ox,self.oy=lo[:2];self.W,self.H=(int((hi[j]-lo[j])/c['res'])+1 for j in (0,1))
        cnt=np.zeros((self.H,self.W),'i4');ij=self._ij(wall);np.add.at(cnt,(ij[:,1],ij[:,0]),1)
        self.occ=cnt>=2;self.dist=cv2.distanceTransform((~self.occ).astype('u1'),cv2.DIST_L2,5)*c['res']
        self.lik=np.exp(-self.dist**2/(2*c['sigma']**2)).astype('f4')
        fm=np.zeros((self.H,self.W),'u1');ij=self._ij(ground);fm[ij[:,1],ij[:,0]]=1
        self.cand=(cv2.dilate(fm,np.ones((7,7),'u1'))>0)&(self.dist>=.15)
        if not self.cand.any():raise ValueError('地图没有可搜索的室内地面候选')
        self.load_s=time.monotonic()-tic

    def _ij(self,P):
        ij=np.floor((P[:,:2]-[self.ox,self.oy])/self.cfg['res']).astype('i8')
        return ij[(ij[:,0]>=0)&(ij[:,0]<self.W)&(ij[:,1]>=0)&(ij[:,1]<self.H)]

    def score(self,xs,ys,th,px,py,lik=None):
        cs,sn=math.cos(th),math.sin(th);rx,ry=cs*px-sn*py,sn*px+cs*py;r=self.cfg['res']
        mx=((xs[:,None]+rx[None,:]-self.ox)/r-.5).astype('f4')
        my=((ys[:,None]+ry[None,:]-self.oy)/r-.5).astype('f4')
        return cv2.remap(self.lik if lik is None else lik,mx,my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0).mean(1)

    def refine2d(self,p,px,py,step):
        x,y,th=p;gx,gy=np.meshgrid(np.linspace(x-step,x+step,11),np.linspace(y-step,y+step,11))
        best=(float(self.score(np.array([x]),np.array([y]),th,px,py)[0]),p)
        for a in th+np.radians(np.linspace(-3,3,13)):
            sc=self.score(gx.ravel(),gy.ravel(),a,px,py);i=int(np.argmax(sc))
            if sc[i]>best[0]:best=(float(sc[i]),(gx.ravel()[i],gy.ravel()[i],a))
        return best

    def global2d(self,px,py):
        c=self.cfg;k=1
        while np.count_nonzero(self.cand[::k,::k])>c['max_cands']:k+=1
        ii,jj=np.nonzero(self.cand[::k,::k]);step=k*c['res']
        cx,cy=self.ox+(jj*k+.5)*c['res'],self.oy+(ii*k+.5)*c['res']
        lik=self.lik if step<=.1+1e-9 else np.exp(-self.dist**2/(2*(step/2)**2)).astype('f4')
        sl=slice(None,None,max(1,len(px)//300));peaks=[]
        for th in np.radians(np.arange(-180,180,c['yaw_step_deg'])):
            sc=self.score(cx,cy,th,px[sl],py[sl],lik).copy()
            # Keep multiple spatial maxima for the SAME yaw, then cluster across yaw.
            for _ in range(3):
                q=int(np.argmax(sc))
                if sc[q]<0:break
                peaks.append((float(sc[q]),cx[q],cy[q],th))
                sc[(cx-cx[q])**2+(cy-cy[q])**2<.5**2]=-1
        peaks.sort(reverse=True);distinct=[]
        for p in peaks:
            if all(math.hypot(p[1]-q[1],p[2]-q[2])>.5 or abs(wrap(p[3]-q[3]))>math.radians(15) for q in distinct):distinct.append(p)
        sl=slice(None,None,max(1,len(px)//1000))
        top=[p for p in distinct[:c['n_candidates']] if p[0]>=c['candidate_ratio']*distinct[0][0]]
        refined=[self.refine2d(p[1:],px[sl],py[sl],max(.1,step)) for p in top]
        return dict(cands=len(cx),step=step,peaks=refined,ratio=distinct[1][0]/max(distinct[0][0],1e-12) if len(distinct)>1 else 0.)

    def zsearch(self,Q,x,y,th,zc,span):
        base=Q@rotz(th).T+[x,y,0]
        def count(z):
            r,_=self.nn.residual(base+[0,0,z],.1);return int((abs(r)<.05).sum())
        z=max(np.arange(zc-span,zc+span+1e-9,.1),key=count)
        return max(np.arange(z-.1,z+.1+1e-9,.02),key=count)

    def icp(self,Q,x0,iters=50,R0=None):
        R=rotz(x0[3]) if R0 is None else np.array(R0,copy=True);t=np.array(x0[:3],float);reach=self.cfg['nn_reach']
        for it in range(iters):
            qr=Q@R.T;dmax=max(.08,reach*.94**it)
            r,n=self.nn.residual(qr+t,dmax);sel=np.isfinite(r)
            if sel.sum()<50:break
            rr,nn,qq=r[sel],n[sel],qr[sel];J=np.c_[nn,np.cross(qq,nn)]
            w=np.minimum(1.,max(.02,dmax/2)/np.maximum(abs(rr),1e-9));H=J.T@(w[:,None]*J)
            try:dx=-np.linalg.solve(H+max(1e-9,np.trace(H)*1e-8)*np.eye(6),J.T@(w*rr))
            except np.linalg.LinAlgError:break
            scale=max(1.,np.linalg.norm(dx[:3])/.2,np.linalg.norm(dx[3:])/math.radians(1))
            dx/=scale;t+=dx[:3];R=exp(dx[3:])@R
            if np.linalg.norm(dx[:3])<1e-5 and np.linalg.norm(dx[3:])<1e-6:break
        qr=Q@R.T;r,n=self.nn.residual(qr+t,reach);cov=np.isfinite(r);inl=abs(r)<.03
        ev=np.zeros(3);rank_ratio=0.
        if cov.sum()>=50:
            nn=n[cov];J=np.c_[nn,np.cross(qr[cov],nn)];H=J.T@J/cov.sum();ev=np.linalg.eigvalsh(H[:3,:3])
            radius=max(1.,np.sqrt(np.mean(np.sum(Q**2,1))));J[:,3:]/=radius
            eig=np.linalg.eigvalsh(J.T@J/len(J));rank_ratio=float(max(0,eig[0])/max(eig[-1],1e-12))
        support=float(np.exp(-r[inl]**2/(2*self.cfg['rmse_ok']**2)).sum()/len(Q)) if len(Q) else 0.
        return dict(R=R,t=t,inlier=float(inl.mean()),support=support,fitness=float(inl.sum()/max(1,cov.sum())),cover=float(cov.mean()),
                    rmse=float(np.sqrt(np.mean(r[inl]**2))) if inl.any() else 1.,iters=it+1,
                    degen=float(ev[0]/max(ev[-1],1e-12)),weakest=float(ev[0]),rank_ratio=rank_ratio,
                    tilt_deg=math.degrees(math.acos(float(np.clip(R[2,2],-1,1)))))


def rank_hypotheses(trials):
    """Rank by observed support before quality gates; weak geometry cannot prove a rival impossible."""
    trials=sorted(trials,key=lambda a:(-a[0]['inlier'],a[0]['rmse']))
    unique=[]
    for ic,score in trials:
        if all(np.linalg.norm(ic['t']-q['t'])>.15 or angle(ic['R']@q['R'].T)>math.radians(3) for q,_ in unique):
            unique.append((ic,score))
    best=unique[0][0]
    # Binary support alone is discontinuous at the 3 cm cutoff. Also compare residual-weighted
    # support using the existing residual tolerance; a better-fitting rival must not disappear.
    ambiguity=max((max(q['inlier']/max(best['inlier'],1e-12),
                       q['support']/max(best['support'],1e-12)) for q,_ in unique[1:]),default=0.)
    return unique,ambiguity


def relocalize(M,cloud,c,hint=None):
    tic=time.monotonic();cfg=M.cfg
    def fail(why):return dict(schema_version=2,ok=False,why=why,secs=time.monotonic()-tic)
    cloud=np.asarray(cloud,float);c=np.asarray(c,float)
    if cloud.ndim!=2 or cloud.shape[1]!=3 or c.shape!=(3,) or not np.isfinite(c).all():return fail('点云或位置格式非法')
    L=cloud-c;L=L[np.isfinite(L).all(1)&(np.linalg.norm(L,axis=1)>=cfg['min_range'])]
    if len(L)<2000:return fail('有效点太少（%d）'%len(L))
    Lv,_=bm.voxel_down(L.astype('f4'),np.zeros(len(L),'f4'),.05);fl=bm.floor_fit(Lv)
    if fl and fl['n']>=200 and .05<fl['height_m']<3:
        h=fl['height_m'];band=(-h+.5,-h+2);zc,span=M.zf+h,.8
    else:h=None;band=(-.8,.6);zc,span=M.zf+1.3,1.3
    wl=Lv[(Lv[:,2]>band[0])&(Lv[:,2]<band[1])]
    if not len(wl):return fail('墙层为空，几何约束不足')
    wall=np.unique(np.floor(wl[:,:2]/cfg['res']).astype('i8'),axis=0)
    if len(wall)<100:return fail('墙层点太少（%d）'%len(wall))
    px,py=((wall+.5)*cfg['res']).T
    use_hint=hint is not None and bool(hint.get('candidates'))
    g=M.global2d(px,py) if not use_hint else dict(cands=0,step=0.,ratio=0.,peaks=[])
    if not use_hint and not g['peaks']:return fail('无全局候选')
    Q,_=bm.voxel_down(L.astype('f4'),np.zeros(len(L),'f4'),.1);Q=Q.astype(float)
    def bad(ic,s2):
        b=[]
        for key,minimum,label in [('inlier',cfg['inlier_ok'],'ICP内点占比'),('fitness',cfg['fit_ok'],'ICP内点率'),('degen',cfg['degen_ok'],'平移退化')]:
            if ic[key]<minimum:b.append('%s %.3f < %.3f'%(label,ic[key],minimum))
        if ic['rank_ratio']<1e-6:b.append('六自由度约束不足')
        if ic['rmse']>cfg['rmse_ok']:b.append('ICP残差超限')
        if ic['tilt_deg']>cfg['max_tilt_deg']:b.append('相对倾角 %.2f° 超限'%ic['tilt_deg'])
        if s2<cfg['score_ok']:b.append('二维得分不足')
        return b
    trials=[]
    starts=[]
    if use_hint:
        from .geometry import unpack
        for candidate in hint['candidates'][:cfg['n_candidates']]:
            R0,tr0=unpack(candidate);p0=R0@c+tr0;th=yaw(R0)
            s2=float(M.score(np.array([p0[0]]),np.array([p0[1]]),th,px,py)[0])
            starts.append((s2,p0,th,R0))
    else:
        for s2,(x,y,th) in g['peaks']:
            starts.append((s2,np.array([x,y,M.zsearch(Q,x,y,th,zc,span)]),th,None))
    for s2,p0,th,R0 in starts:
        seeds=[M.icp(Q,[*p0,th+math.radians(a)],R0=rotz(math.radians(a))@R0 if R0 is not None else None) for a in (0,-1,1)]
        ic=max(seeds,key=lambda a:(a['inlier'],-a['rmse']))
        trials.append((ic,s2))
    uniq,ambiguity=rank_hypotheses(trials)
    ic,s2=uniq[0]
    reasons=bad(ic,s2)
    if ambiguity>cfg['amb_max']:reasons.append('竞争解歧义 %.3f > %.3f'%(ambiguity,cfg['amb_max']))
    R,t=ic['R'],ic['t']-ic['R']@c
    metrics={k:v for k,v in ic.items() if k not in ('R','t')}
    return dict(schema_version=2,ok=not reasons,why='；'.join(reasons),transform=transform(R,t),
                T=[*t.tolist(),wrap(yaw(R))],T_is_yaw_projection=True,drone_map=ic['t'].tolist(),
                height=float(ic['t'][2]-M.zf),floor_local=h,score2d=s2,ratio2d=g['ratio'],amb3d=ambiguity,
                n_cand=len(trials),cands=g['cands'],step=g['step'],n_icp=len(Q),n_wall=len(wall),
                candidates=[dict(transform=transform(a['R'],a['t']-a['R']@c),inlier=a['inlier'],support=a['support'],fitness=a['fitness'],
                                 degen=a['degen'],rmse=a['rmse'],tilt_deg=a['tilt_deg'],why=bad(a,s)) for a,s in uniq],
                search_mode='confirm_candidates' if use_hint else 'global',secs=time.monotonic()-tic,**metrics)


