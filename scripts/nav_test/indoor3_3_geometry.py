"""Tag observation quality and fixed map-yaw camera standoff geometry."""
import math
import numpy as np
from drone_navigation.core import finite, rotation, heading, rz


def map_tag(nav, tag_pose, reference, cam_translation, cam_rotation, lever):
    r, t = reference
    body = r @ rotation(nav.quaternion)
    camera = r @ nav.position + t + body @ (cam_translation - lever)
    p, q = tag_pose.position, tag_pose.orientation
    cam_tag = finite([p.x, p.y, p.z])
    tag_rotation = body @ cam_rotation @ rotation([q.x, q.y, q.z, q.w])
    if cam_tag[2] <= 0 or np.dot(tag_rotation[:, 2], -body @ cam_rotation @ cam_tag) <= 0:
        raise ValueError('tag normal does not face camera')
    if abs(tag_rotation[2, 2]) > math.sin(math.radians(15)) or tag_rotation[2, 1] < math.cos(math.radians(20)):
        raise ValueError('label is not upright on a vertical face')
    return camera + body @ cam_rotation @ cam_tag, tag_rotation, camera


def consolidate(samples):
    if len(samples) < 3:
        raise ValueError('fewer than 3 time-aligned observations')
    points = np.asarray([sample[0] for sample in samples])
    center = np.median(points, axis=0)
    normals = np.asarray([sample[1][:, 2] for sample in samples])
    normal = normals.mean(axis=0)
    normal /= max(np.linalg.norm(normal), 1e-9)
    if np.max(np.linalg.norm(points - center, axis=1)) > .10:
        raise ValueError('tag positions vary by more than 0.10 m from the median')
    if np.min(normals @ normal) < math.cos(math.radians(10)):
        raise ValueError('tag normals vary by more than 10 degrees from the mean')
    # Rebuild an orthonormal frame while preserving tag-up for the paper offset.
    up = np.asarray([sample[1][:, 1] for sample in samples]).mean(axis=0)
    right = np.cross(up, normal)
    right /= np.linalg.norm(right)
    return center, np.column_stack((right, np.cross(normal, right), normal))


def fixed_scan_goal(tag_center, axes, reference, camera_lever, yaw_map):
    r, t = reference
    camera_map = tag_center + .8 * axes[:, 2]
    yaw_odom = heading(r.T, yaw_map)
    control_odom = r.T @ (camera_map - t) - rz(yaw_odom) @ camera_lever
    return r @ control_odom + t, camera_map
