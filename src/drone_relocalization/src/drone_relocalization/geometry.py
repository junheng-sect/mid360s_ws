"""SE(3) conventions: column vectors, xyzw quaternions, left multiplication."""
import math
import cv2
import numpy as np


def exp(v):
    return cv2.Rodrigues(np.asarray(v, float).reshape(3))[0]


def quat_R(q):
    q = np.asarray(q, float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > .02:
        raise ValueError('invalid xyzw quaternion')
    x,y,z,w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def R_quat(R):
    v = cv2.Rodrigues(np.asarray(R, float))[0].ravel()
    a = np.linalg.norm(v)
    return np.r_[v*(math.sin(a/2)/a) if a>1e-12 else v/2, math.cos(a/2)].tolist()


def angle(R):
    return float(np.linalg.norm(cv2.Rodrigues(np.asarray(R,float))[0]))


def yaw(R):
    return math.atan2(R[1,0],R[0,0])


def wrap(a):
    return (a+math.pi)%(2*math.pi)-math.pi


def rotz(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])


def transform(R,t):
    return dict(translation=np.asarray(t,float).tolist(),q_xyzw=R_quat(R))


def unpack(r):
    a=r['transform'];return quat_R(a['q_xyzw']),np.array(a['translation'],float)


def distance(a,b):
    ra,ta=unpack(a);rb,tb=unpack(b)
    return float(np.linalg.norm(ta-tb)),math.degrees(angle(ra@rb.T))


def mapped_pose(result,Rwc,twc,Rib,tib,pose):
    """pose = [stamp,p_cam(3),q_cam_body(4)]. base origin follows recorded body->base transform."""
    R,t=unpack(result);Rb=quat_R(pose[4:8]);p=Rwc@(pose[1:4]+Rb@tib)+twc
    return R@p+t,R@Rwc@Rb@Rib
