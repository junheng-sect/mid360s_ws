#!/usr/bin/env python3
"""Hardware-free numerical acceptance on the board; outputs inspectable JSON."""
import argparse,json,math,os,time,resource,tempfile
import numpy as np
from drone_mapping import mapio
from drone_relocalization.geometry import exp,rotz,unpack,angle
from drone_relocalization.matching import MapModel,relocalize
from drone_relocalization.session import BagSession,run_once
from drone_relocalization.offline import startup


def main():
    a=argparse.ArgumentParser();a.add_argument('--out',required=True);args=a.parse_args();base=os.path.expanduser('~/test_data/mapping')
    M=MapModel(base+'/20261018_194304_voxelslam');S=BagSession(base+'/flight_raw_000001.bag')
    points=S.cloud(1,2);center,_=S.pose_at(S.t0+2);ref=relocalize(M,points,center)
    if not ref['ok']:raise RuntimeError('reference not trusted: '+ref['why'])
    R,t=unpack(ref);pmap=points@R.T+t;cmap=R@center+t;cases=[]
    tests=[(-180,0,0),(-135,1,0),(-90,-1,0),(-45,0,1),(0,0,-1),(45,2,0),(90,-2,0),(135,0,2),(179,0,-2)]
    for y,roll,pitch in tests:
        expected=rotz(math.radians(y))@exp(np.radians([roll,pitch,0]));translation=np.array([1.3,-.8,.6])
        # Re-express the SAME observed cloud in a synthetic world frame. Expected map<-world is known exactly relative to ref.
        source=(pmap-translation)@expected;c=(cmap-translation)@expected
        r=relocalize(M,source,c);er=ea=None
        if 'transform' in r:
            got,gt=unpack(r);er=float(np.linalg.norm(gt-translation));ea=math.degrees(angle(got@expected.T))
        cases.append(dict(yaw=y,roll=roll,pitch=pitch,ok=r['ok'],translation_error_m=er,rotation_error_deg=ea,
                          passed=bool(r['ok'] and er<=.02 and ea<=.3),why=r['why'],secs=r['secs']))
        print('PERTURB',cases[-1],flush=True)
    # Tilt budget must reject, not silently clip to 3 degrees.
    expected=exp(np.radians([5.,0,0]));r=relocalize(M,pmap@expected,cmap@expected)
    overtilt=dict(passed=not r['ok'],ok=r['ok'],why=r['why'],tilt=r.get('tilt_deg'))
    S.close()
    # Exact synthetic rectangular room has genuine yaw/spatial ambiguity, independent of recorded-room labels.
    with tempfile.TemporaryDirectory(prefix='reloc-symmetric-') as d:
        u=np.arange(-2,2.001,.08);v=np.arange(-1.5,1.501,.08);z=np.arange(-.15,2.651,.08)
        xx,yy=np.meshgrid(u,v);parts=[np.c_[xx.ravel(),yy.ravel(),np.full(xx.size,h)] for h in [-.15,2.65]]
        for x in [-2,2]:
            yy,zz=np.meshgrid(v,z);parts.append(np.c_[np.full(yy.size,x),yy.ravel(),zz.ravel()])
        for y in [-1.5,1.5]:
            xx,zz=np.meshgrid(u,z);parts.append(np.c_[xx.ravel(),np.full(xx.size,y),zz.ravel()])
        room=np.concatenate(parts).astype('f4');cloud=room.copy();both=np.r_[room,room+[8,0,0]].astype('f4')
        mapio.write_pcd(d+'/map_world.pcd',both,np.zeros(len(both),'f4'))
        with open(d+'/meta.json','w') as f:json.dump(dict(floor=dict(height_m=.15,trusted=True)),f)
        mm=MapModel(d);sym=relocalize(mm,cloud,np.array([.3,.2,1.]));symmetric=dict(passed=not sym['ok'],ok=sym['ok'],why=sym['why'],amb3d=sym.get('amb3d'),n_cand=sym.get('n_cand'))
        print('SYMMETRIC',symmetric,flush=True)
    # Historical failure windows must be rejected; no falling back from a stronger degenerate hypothesis.
    known_wrong=[]
    for bag,times in [('20261018_194304/raw.bag',[21.,51.]),('20261018_192355/raw.bag',[16.,51.,76.])]:
        wrong=BagSession(base+'/'+bag)
        for at in times:
            r=run_once(M,wrong,at,2.)
            known_wrong.append(dict(bag=bag,at=at,passed=not r['ok'],ok=r['ok'],why=r['why'],transform=r.get('transform')))
            print('KNOWN_WRONG',known_wrong[-1],flush=True)
        wrong.close()
    starts={}
    for name in ['20261018_192355/raw.bag','20261018_194304/raw.bag','flight_raw_000001.bag']:
        ss=BagSession(base+'/'+name);starts[name]=startup(M,ss,1.);ss.close()
        print('STARTUP',name,starts[name]['ok'],starts[name]['elapsed_data_s'],starts[name]['why'],flush=True)
    output=dict(perturbations=cases,overtilt=overtilt,symmetric=symmetric,known_wrong=known_wrong,startup=starts,
                max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    with open(args.out,'w') as f:json.dump(output,f,ensure_ascii=False,indent=2)
    assert all(x['passed'] for x in cases), 'known-transform consistency acceptance failed'
    assert overtilt['passed'] and symmetric['passed'] and all(x['passed'] for x in known_wrong),'negative acceptance failed'


if __name__=='__main__':main()
