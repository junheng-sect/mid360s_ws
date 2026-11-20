"""Indoor test4 geometry and exact per-observation route ordering."""
import math

import numpy as np

from drone_navigation.core import finite, heading, rotation, rz


STANDOFF = .8


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


def scan_target(tag_position, tag_rotation, label, reference, camera_lever, yaw_map):
    """Place the optical center in front of the paper with the route heading in drone_map."""
    paper = tag_position - tag_rotation @ np.array([label['tag_offset_x_meters'],
                                                    label['tag_offset_y_meters'], 0.])
    normal = tag_rotation[:, 2]
    camera = paper + STANDOFF * normal
    r, t = reference
    yaw_odom = heading(r.T, yaw_map)
    point_odom = r.T @ (camera - t) - rz(yaw_odom) @ camera_lever
    return r @ point_odom + t, point_odom, yaw_map, camera, paper


def shortest_order(start, points, finish):
    """Return the ID order and total 3D Euclidean distance, including the fixed finish."""
    keys = sorted(points)
    n = len(keys)
    if n > 10:
        raise ValueError('more than 10 distinct tags at one observation point')
    if not n:
        return (), float(np.linalg.norm(start - finish))
    p = np.asarray([points[key] for key in keys])
    distances = np.linalg.norm(p[:, None] - p[None, :], axis=2)
    initial = np.linalg.norm(p - start, axis=1)
    final = np.linalg.norm(p - finish, axis=1)
    dp = {(1 << i, i): (initial[i], (keys[i],)) for i in range(n)}
    for mask in range(1, 1 << n):
        for i in range(n):
            if not mask & (1 << i) or mask == 1 << i:
                continue
            previous = mask ^ (1 << i)
            dp[mask, i] = min((dp[previous, j][0] + distances[j, i],
                               dp[previous, j][1] + (keys[i],))
                              for j in range(n) if previous & (1 << j))
    distance, order = min((dp[(1 << n) - 1, i][0] + final[i], dp[(1 << n) - 1, i][1])
                          for i in range(n))
    return order, float(distance)
