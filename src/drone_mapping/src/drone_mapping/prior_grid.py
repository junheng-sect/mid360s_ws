"""Generate and validate the versioned EGO occupancy prior for a Voxel-SLAM map."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

import numpy as np

from drone_mapping import assemble, mapio


MAGIC = 'EGO_PRIOR_1'
FRAME = 'drone_map'
RESOLUTION = 0.05
DEFAULT_BOUNDS = (-8, -8, -2, 8, 8, 5)
MAX_CELLS = 25_000_000
MAX_HEADER = 1024


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def geometry(origin, shape):
    if (not all(math.isfinite(x) and abs(x) <= 10000 for x in origin)
            or not all(1 <= n <= 1000 for n in shape) or math.prod(shape) > MAX_CELLS):
        raise ValueError('先验栅格尺寸或原点非法')


def verify(folder):
    path = folder / 'prior_grid.bin'
    with path.open('rb') as stream:
        header = stream.readline(MAX_HEADER + 1)
        if len(header) > MAX_HEADER or not header.endswith(b'\n'):
            raise ValueError('先验栅格文件头损坏')
        fields = header.decode('ascii').split()
        if len(fields) != 15 or fields[:2] != [MAGIC, FRAME] or float(fields[2]) != RESOLUTION:
            raise ValueError('先验栅格版本、坐标系或分辨率不符')
        origin, shape = list(map(float, fields[3:6])), list(map(int, fields[6:9]))
        geometry(origin, shape)
        if not all(re.fullmatch('[0-9a-f]{64}', field) for field in fields[9:]):
            raise ValueError('先验栅格缺少源文件校验值')
        map_sha, payload_sha = fields[9:11]
        payload = stream.read(math.prod(shape) + 1)
        if (len(payload) != math.prod(shape) or hashlib.sha256(payload).hexdigest() != payload_sha
                or np.frombuffer(payload, dtype=np.uint8).max() > 2):
            raise ValueError('先验栅格内容校验失败')
    if sha256(folder / 'map_world.pcd') != map_sha:
        raise ValueError('先验栅格与当前 map_world.pcd 的 SHA-256 不符')
    return map_sha


def build(folder, bag_override, bounds):
    meta_path = folder / 'meta.json'
    pcd_path = folder / 'map_world.pcd'
    config_path = folder / 'voxelslam.yaml'
    meta = json.loads(meta_path.read_text())
    if meta.get('algo') != 'voxelslam':
        raise ValueError('需要 Voxel-SLAM 原始建图包和位姿')
    state_path = folder / meta['voxelslam']['state']
    bag_path = bag_override or Path(meta['source_bag'])
    world = meta['world_from_voxelslam']
    rotation = mapio.quat_R(*world['q_xyzw'])
    translation = np.asarray(world['t'], dtype=float)
    traj, _ = assemble.read_state(state_path)
    if (not np.isfinite(rotation).all() or translation.shape != (3,)
            or not np.isfinite(translation).all() or len(traj) < 2
            or not np.isfinite(traj).all() or not np.all(np.diff(traj[:, 0]) > 0)):
        raise ValueError('建图位姿或地图变换非法')
    params = assemble.load_params(config_path)
    if params['every'] < 1 or not np.isfinite(params['R']).all() or not np.isfinite(params['T']).all():
        raise ValueError('建图雷达参数非法')

    limits = np.asarray(bounds).reshape(2, 3) / RESOLUTION
    if not np.isfinite(limits).all() or not np.allclose(limits, np.rint(limits), atol=1e-7, rtol=0):
        raise ValueError('先验边界必须对齐到 5 cm 网格')
    origin = np.rint(limits[0]) * RESOLUTION
    shape = tuple(int(n) for n in np.rint(limits[1] - limits[0]))
    geometry(origin, shape)
    count = 0
    with tempfile.TemporaryFile() as ray_result:
        cmd = ['rosrun', 'drone_mapping', 'prior_grid_raycast',
               *(str(x) for x in (*origin, *shape, params['blind']))]
        with subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=ray_result) as worker:
            try:
                for xyz, _, times in assemble.iter_scan_points(str(bag_path), traj, params):
                    rays = np.empty((len(xyz), 7), dtype=np.float64)
                    offsets = np.broadcast_to(params['T'], xyz.shape)
                    imu_points = xyz @ params['R'].T + offsets
                    rays[:, :3] = assemble.to_world(offsets, times, traj, rotation) + translation
                    rays[:, 3:6] = assemble.to_world(imu_points, times, traj, rotation) + translation
                    rays[:, 6] = np.floor(times - traj[0, 0])
                    worker.stdin.write(rays.tobytes())
                    count += 1
            except BaseException:
                worker.kill()
                raise
            finally:
                worker.stdin.close()
            if worker.wait() != 0:
                raise ValueError('离线射线遍历失败')
        ray_result.seek(0)
        cells = np.frombuffer(ray_result.read(), dtype=np.uint8).copy()
    if count != meta['lidar_msgs_used'] or cells.size != math.prod(shape):
        raise ValueError('重放帧数或栅格大小与建图记录不符 (%s 帧)' % count)

    points, _ = mapio.read_pcd(pcd_path)
    points = points[np.isfinite(points).all(axis=1)]
    indices = np.floor((points - origin) / RESOLUTION).astype(np.int64)
    indices = indices[((indices >= 0) & (indices < shape)).all(axis=1)]
    cells.reshape(shape)[tuple(indices.T)] = 2
    if not (cells == 1).any() or not (cells == 2).any():
        raise ValueError('地图范围内缺少有效空闲或占用证据')

    hashes = [sha256(path) for path in (pcd_path, bag_path, state_path, config_path, meta_path)]
    hashes.insert(1, hashlib.sha256(cells).hexdigest())
    header = ' '.join(map(str, (MAGIC, FRAME, RESOLUTION, *origin, *shape, *hashes))) + '\n'
    out = folder / 'prior_grid.bin'
    fd, tmp = tempfile.mkstemp(prefix='.prior_grid-', dir=str(folder))
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(header.encode('ascii'))
            stream.write(cells)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, out)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
    totals = np.bincount(cells, minlength=3)
    print('%s 帧，未知 %s，空闲 %s，占用 %s。%s' % (count, *totals, out), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_dir', type=Path)
    parser.add_argument('--bag', type=Path, help='原始包移到别处时指定路径')
    parser.add_argument('--bounds', nargs=6, type=float, default=DEFAULT_BOUNDS,
                        metavar=('XMIN', 'YMIN', 'ZMIN', 'XMAX', 'YMAX', 'ZMAX'))
    parser.add_argument('--verify', action='store_true', help='检查先验文件和当前地图，输出地图 SHA-256')
    args = parser.parse_args()
    try:
        if args.verify:
            print(verify(args.map_dir.resolve()))
        else:
            build(args.map_dir.resolve(), args.bag, args.bounds)
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        sys.exit('先验栅格 %s: %s' % (args.map_dir, error))
