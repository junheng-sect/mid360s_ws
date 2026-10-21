"""Unified map calibration (config/map_frame.yaml) and planar SE(3) helpers; column vectors, left multiplication."""
import math
import os
import numpy as np
import yaml

KEYS={'drone_map','car_map_id','map_from_drone_map','map_from_car_map','car_base_height'}


def se2(x,y,z,yaw_deg):
    a=math.radians(yaw_deg);c,s=math.cos(a),math.sin(a)
    return np.array([[c,-s,0,x],[s,c,0,y],[0,0,1,z],[0,0,0,1.]])


def yaw(T):
    return math.atan2(T[1,0],T[0,0])


def quat_yaw(a):
    """xyzw quaternion of a rotation about +z."""
    return 0.,0.,math.sin(a/2),math.cos(a/2)


def load(path):
    with open(os.path.expanduser(path)) as f:d=yaml.safe_load(f)
    if not isinstance(d,dict) or set(d)!=KEYS:raise ValueError('map_frame keys must be exactly %s'%sorted(KEYS))
    T={k:se2(*(float(d[k][a]) for a in ('x','y','z','yaw_deg'))) for k in ('map_from_drone_map','map_from_car_map')}
    return dict(drone_map=str(d['drone_map']),car_map_id=int(d['car_map_id']),height=float(d['car_base_height']),
                map_drone=T['map_from_drone_map'],map_car=T['map_from_car_map'],
                car_drone=np.linalg.inv(T['map_from_car_map'])@T['map_from_drone_map'])


def pose_xyyaw(msg):
    """x, y, yaw of a rosbridge geometry_msgs/PoseStamped dict."""
    p,q=msg['pose']['position'],msg['pose']['orientation']
    return p['x'],p['y'],math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))


def car_base(F,pose):
    """car_map<-car_base: the car's tracked pose raised to car_base_height (car lidar height)."""
    x,y,th=pose_xyyaw(pose);return se2(x,y,F['height'],math.degrees(th))
