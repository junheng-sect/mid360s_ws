import argparse
import json
import os
import sys
import time
import numpy as np


def ensure_ros_env(entry):
    # Source setups only to obtain Python dependencies; never overwrite ROS network variables.
    try:import rosbag  # noqa: F401
    except ImportError:
        if os.environ.get('DRONE_RELOC_ENV')=='1':raise
        root=os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))))
        setup=os.path.join(root,'devel/setup.bash')
        command=('reloc_entry="$0"; reloc_setup="$1"; shift; reloc_args=("$@"); set --; '
                 'source /opt/ros/noetic/setup.bash && source "$reloc_setup" && export DRONE_RELOC_ENV=1 '
                 '&& exec python3 "$reloc_entry" "${reloc_args[@]}"')
        os.execvp('bash',['bash','-c',command,entry,setup]+sys.argv[1:])


def main(entry=None):
    if entry:ensure_ros_env(entry)
    from .settings import load
    from .matching import MapModel
    from .session import BagSession,run_once
    from .offline import startup
    p=argparse.ArgumentParser(description='一次性SE3全局重定位（室内小图；室外大图按起飞区裁图），双静止窗口确认，不接飞控')
    p.add_argument('--map',required=True);p.add_argument('--config',default='')
    p.add_argument('--bag',default='');p.add_argument('--at',type=float,default=1.)
    p.add_argument('--eval',action='store_true');p.add_argument('--startup-eval',action='store_true')
    p.add_argument('--step',type=float,default=5.);p.add_argument('--yaw-perturb',action='store_true')
    p.add_argument('--window',type=float,default=None);p.add_argument('--tries',type=int,default=None)
    p.add_argument('--still-timeout',type=float,default=None)
    p.add_argument('--out',default='');p.add_argument('--save',default=os.path.expanduser('~/test_data/reloc'))
    import rospy
    args=p.parse_args(rospy.myargv()[1:])
    if args.at<0 or args.step<=0 or args.eval and args.startup_eval:p.error('invalid time/evaluation arguments')
    if (args.eval or args.startup_eval or args.yaw_perturb) and not args.bag:p.error('offline options require --bag')
    args.map=os.path.abspath(os.path.expanduser(args.map));args.startup_started=time.monotonic()
    try:
        cfg=load(args.config or None,dict(window=args.window,tries=args.tries,timeout=args.still_timeout))
        M=MapModel(args.map,cfg)
        print('地图 %d 点%s，载入 %.2fs，cache=%s'%(len(M.P),'（起飞区 %g m 内裁图，起飞圆柱 r=%g m）'%(cfg['crop_radius'],cfg['zone_radius']) if cfg['crop_radius'] else '',M.load_s,M.cache_hit),flush=True)
        if not args.bag:
            from .runtime import online
            return online(M,args)
        S=BagSession(os.path.expanduser(args.bag))
        try:
            if args.startup_eval:
                result=startup(M,S,args.at);print(json.dumps(result,ensure_ascii=False),flush=True);ok=result['ok']
            else:
                results=[];rng=np.random.RandomState(0)
                times=np.arange(args.at,S.dur-cfg['window'],args.step) if args.eval else [args.at]
                for t in times:
                    r=run_once(M,S,float(t),cfg['window'],float(rng.uniform(-np.pi,np.pi)) if args.yaw_perturb else 0.)
                    results.append(r);print('t=%.1f %s fitness=%.3f degen=%.3f tilt=%.2f rmse=%.4f %.2fs %s'%
                        (t,'可信' if r['ok'] else '拒绝',r.get('fitness',0),r.get('degen',0),r.get('tilt_deg',0),r.get('rmse',1),r['secs'],r['why']),flush=True)
                ok=bool(results) and (any(r['ok'] for r in results) if args.eval else results[-1]['ok'])
                result=dict(schema_version=2,map=args.map,bag=args.bag,window=cfg['window'],results=results,
                            summary=dict(trusted=sum(r['ok'] for r in results),total=len(results),
                                         median_s=float(np.median([r['secs'] for r in results])) if results else None))
                print(json.dumps(result['summary']),flush=True)
            if args.out:
                os.makedirs(os.path.dirname(os.path.abspath(args.out)),exist_ok=True)
                with open(args.out,'w') as f:json.dump(result,f,ensure_ascii=False,indent=2)
            return 0 if ok else 1
        finally:S.close()
    except (ValueError,OSError,KeyError) as exc:
        print('[ERROR] '+str(exc),file=sys.stderr);return 3
