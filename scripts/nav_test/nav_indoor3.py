#!/usr/bin/env python3
"""Photograph four fixed shelf positions through the standard global test runner."""

import argparse
import hashlib
from pathlib import Path

import yaml


ROUTE_FILE = Path(__file__).with_name('indoor3_route.yaml')


def load_route():
    return yaml.safe_load(ROUTE_FILE.read_text())


def verify_map(path, route):
    path = path.resolve()
    if path.name != route['map_id']:
        raise ValueError('室内 test3 需要地图 ' + route['map_id'])
    with (path / 'map_world.pcd').open('rb') as stream:
        digest = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    if digest.hexdigest() != route['map_sha256']:
        raise ValueError('室内 test3 地图 SHA-256 与航点定义不一致')


def run_ros(args, route):
    import rospy
    from indoor3_runner import Indoor3Runner, probe_photo, serve_route
    if args.probe_camera:
        rospy.init_node('indoor3_camera_probe', anonymous=True)
        return probe_photo(args.out)
    return serve_route('nav_indoor3', args, route, Indoor3Runner)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify-map', type=Path)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--probe-camera', action='store_true')
    parser.add_argument('--preview', action='store_true')
    args = parser.parse_args()
    route = load_route()
    if args.verify_map is not None:
        verify_map(args.verify_map, route)
        return 0
    if args.out is None:
        parser.error('--out is required')
    return run_ros(args, route)


if __name__ == '__main__':
    raise SystemExit(main())
