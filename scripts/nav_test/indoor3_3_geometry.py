"""Tag observation quality and fixed map-yaw camera standoff geometry."""
import math
import numpy as np
from indoor3_3_lock import LockRejection, MIN_SAMPLES
from drone_navigation.core import finite, rotation, heading, rz


def map_tag(nav, tag_pose, reference, cam_translation, cam_rotation, lever):
    r, t = reference
    body = r @ rotation(nav.quaternion)
    camera = r @ nav.position + t + body @ (cam_translation - lever)
    p, q = tag_pose.position, tag_pose.orientation
    cam_tag = finite([p.x, p.y, p.z])
    tag_rotation = body @ cam_rotation @ rotation([q.x, q.y, q.z, q.w])
    if cam_tag[2] <= 0:
        raise LockRejection('behind_camera', float(cam_tag[2]))
    facing = float(np.dot(tag_rotation[:, 2], -body @ cam_rotation @ cam_tag))
    if facing <= 0:
        raise LockRejection('back_facing', facing)
    if abs(tag_rotation[2, 2]) > math.sin(math.radians(15)):
        raise LockRejection('normal_tilt', math.degrees(math.asin(min(1., abs(tag_rotation[2, 2])))))
    if tag_rotation[2, 1] < math.cos(math.radians(20)):
        raise LockRejection('tag_up', math.degrees(math.acos(float(np.clip(tag_rotation[2, 1], -1., 1.)))))
    return camera + body @ cam_rotation @ cam_tag, tag_rotation, camera


def consolidate(samples):
    if len(samples) < MIN_SAMPLES:
        raise LockRejection('too_few', len(samples))
    points = np.asarray([sample[0] for sample in samples])
    center = np.median(points, axis=0)
    normals = np.asarray([sample[1][:, 2] for sample in samples])
    normal = normals.mean(axis=0)
    normal /= max(np.linalg.norm(normal), 1e-9)
    spread = float(np.max(np.linalg.norm(points - center, axis=1)))
    if spread > .10:
        raise LockRejection('position_spread', spread)
    min_cos = float(np.min(normals @ normal))
    if min_cos < math.cos(math.radians(10)):
        raise LockRejection('normal_spread', math.degrees(math.acos(float(np.clip(min_cos, -1., 1.)))))
    # Rebuild an orthonormal frame while preserving tag-up for the paper offset.
    up = np.asarray([sample[1][:, 1] for sample in samples]).mean(axis=0)
    right = np.cross(up, normal)
    right /= np.linalg.norm(right)
    return center, np.column_stack((right, np.cross(normal, right), normal))


STANDOFF_M = 0.80


def fixed_scan_goal(tag_center, axes, reference, camera_lever, yaw_map):
    r, t = reference
    camera_map = tag_center + STANDOFF_M * axes[:, 2]
    yaw_odom = heading(r.T, yaw_map)
    control_odom = r.T @ (camera_map - t) - rz(yaw_odom) @ camera_lever
    return r @ control_odom + t, camera_map
