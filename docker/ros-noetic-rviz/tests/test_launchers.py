#!/usr/bin/env python3
"""Offline regression checks for separate Mac entry points, preserved Ubuntu files and dry-run effects."""
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
EGO = Path(os.environ.get('EGO_RVIZ_SOURCE', str(Path.home() / 'Projects/Orange/tools/ego_rviz')))


class LaunchTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.work = Path(self.temp.name)
        self.shared = self.work / 'shared with spaces'
        shutil.copytree(ROOT, self.shared, ignore=shutil.ignore_patterns('home', 'runtime', 'logs', '__pycache__'))
        self.ego = self.work / 'ego with spaces'
        for directory in ('scripts', 'rviz'):
            shutil.copytree(EGO / directory, self.ego / directory)
        self.bin = self.work / 'bin'
        self.bin.mkdir()
        self.calls = self.work / 'calls'
        for name in ('docker', 'sudo', 'xauth', 'xhost', 'open'):
            file = self.bin / name
            file.write_text('#!/bin/sh\nprintf "%s\\n" "$0 $*" >> "$CALLS"\nexit 99\n')
            file.chmod(0o755)

    def tearDown(self):
        self.temp.cleanup()

    def launch(self, arguments=(), platform='Darwin', ego=False, env_extra=None):
        uname = self.bin / 'uname'
        uname.write_text('#!/bin/sh\nprintf "%s\\n" ' + platform + '\n')
        uname.chmod(0o755)
        env = dict(os.environ, PATH=str(self.bin) + ':' + os.environ['PATH'], CALLS=str(self.calls),
                   BOARD_IP='192.168.31.32', EGO_RVIZ_SHARED=str(self.shared))
        for key in ('ROS_MASTER_URI', 'RVIZ_IMAGE_TAG', 'IMAGE_TAG', 'RVIZ_CONFIG', 'RVIZ_WEB_PORT'):
            env.pop(key, None)
        env.update(env_extra or {})
        script = self.ego / 'scripts/run_mac.sh' if ego else self.shared / 'scripts/run_rviz_mac.sh'
        return subprocess.run(['/bin/bash', str(script), '--dry-run', '--ros-ip', '192.168.31.50', *arguments],
                              env=env, text=True, capture_output=True, timeout=10)

    def test_mac_display_paths(self):
        for ego in (False, True):
            result = self.launch(ego=ego)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('linux/arm64', result.stdout)
            self.assertIn('/usr/local/bin/rviz-web-desktop', result.stdout)
            self.assertNotIn('--gpus', result.stdout)
            self.assertNotIn('/tmp/.X11-unix', result.stdout)
        self.assertFalse(self.calls.exists(), 'dry-run called Docker or a mutating helper')
        for root in (self.shared, self.ego):
            for directory in ('home', 'runtime', 'logs'):
                self.assertFalse((root / directory).exists())

    def test_mac_entry_rejects_other_hosts_without_effects(self):
        for ego in (False, True):
            result = self.launch(platform='Linux', ego=ego)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('Mac 专用入口', result.stderr)
        self.assertFalse(self.calls.exists())

    def test_original_ubuntu_files_unchanged(self):
        # Exact file preservation catches changes that command snapshots can miss.
        for repo, names in (
            (ROOT.parents[1], ('scripts/run_rviz.sh',
              'docker/ros-noetic-rviz/scripts/run_rviz.sh',
              'docker/ros-noetic-rviz/scripts/common.sh',
              'docker/ros-noetic-rviz/scripts/build_image.sh',
              'docker/ros-noetic-rviz/Dockerfile',
              'docker/ros-noetic-rviz/.dockerignore',
              'docker/ros-noetic-rviz/local.env.example',
              'docker/ros-noetic-rviz/launch/display_gravity_aligned.launch',
              'docker/ros-noetic-rviz/launch/display_reloc.launch')),
            (EGO.parents[1], ('tools/ego_rviz/scripts/run.sh',
              'tools/ego_rviz/scripts/check_inputs.py', 'tools/ego_rviz/Dockerfile'))):
            for name in names:
                original = subprocess.check_output(['git', 'show', 'HEAD:' + name], cwd=str(repo))
                self.assertEqual((repo / name).read_bytes(), original, name)

    def test_mac_ignores_ubuntu_configuration(self):
        (self.shared / 'local.env').write_text('RVIZ_IMAGE_TAG=ubuntu_only\nRVIZ_CONFIG=/home/ros/ubuntu.rviz\nRVIZ_WEB_PORT=9999\n')
        for ego in (False, True):
            result = self.launch(ego=ego, env_extra={'IMAGE_TAG': 'ubuntu_environment'})
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertNotIn('ubuntu_', result.stdout)
            self.assertNotIn('9999', result.stdout)
        self.assertFalse(self.calls.exists())

    def test_headless_and_stop_do_not_start_desktop(self):
        for argument in ('--check', '--shell'):
            result = self.launch((argument,))
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertNotIn('rviz-web-desktop', result.stdout)
            self.assertNotIn('--gpus', result.stdout)
            self.assertNotIn('DISPLAY=', result.stdout)
        result = self.launch(('--stop', '--name', 'my_view'))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), 'docker stop my_view')
        self.assertFalse(self.calls.exists())

    def test_configuration_precedence_and_relocation(self):
        (self.shared / 'local.mac.env').write_text('RVIZ_CONFIG="/home/ros/custom.rviz"\nRVIZ_WEB_PORT=6090\n')
        result = self.launch()
        self.assertIn('/home/ros/custom.rviz', result.stdout)
        self.assertIn('RVIZ_WEB_PORT=6090', result.stdout)
        result = self.launch(('--reloc', '--web-port', '6092'))
        self.assertIn('/home/ros/.rviz/drone_reloc.rviz', result.stdout)
        self.assertIn('display_reloc_mac.launch', result.stdout)
        self.assertIn('RVIZ_WEB_PORT=6092', result.stdout)
        result = self.launch(('--reloc', '--config', '/home/ros/explicit.rviz'))
        self.assertIn('/home/ros/explicit.rviz', result.stdout)
        self.assertNotIn('/home/ros/.rviz/drone_reloc.rviz', result.stdout)

    def test_ego_map_and_selected_odometry(self):
        mapping = self.work / 'map with spaces'
        mapping.mkdir()
        (mapping / 'map_world.pcd').write_text('fixture')
        result = self.launch(('--map', str(mapping), '--odom', '/fastlio/odom_world'), ego=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('/maps/selected.pcd:ro', result.stdout)
        self.assertIn('ego_swarm_map.rviz', result.stdout)
        self.assertIn('odom:=/fastlio/odom_world', result.stdout)
        result = self.launch(('--check', '--no-map', '--odom', '/fastlio/odom_world'), ego=True)
        self.assertIn('--odom /fastlio/odom_world', result.stdout)
        self.assertIn('--fixed-frame drone_odom', result.stdout)

    def test_invalid_arguments_are_action_free(self):
        for arguments in (('--web-port', '65535'), ('--web-port', 'abc'), ('--config',), ('--ros-master', 'bad')):
            result = self.launch(arguments)
            self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.calls.exists())

    def test_mac_image_build_paths(self):
        docker = self.bin / 'docker'
        docker.write_text('#!/bin/sh\nprintf "%s\\n" "$*" >> "$CALLS"\n'
                          'case "$1" in image) printf "sha256:test\\n";; esac\nexit 0\n')
        uname = self.bin / 'uname'
        uname.write_text('#!/bin/sh\nprintf "%s\\n" Darwin\n')
        uname.chmod(0o755)
        env = dict(os.environ, PATH=str(self.bin) + ':' + os.environ['PATH'], CALLS=str(self.calls),
                   BOARD_IP='192.168.31.32', EGO_RVIZ_SHARED=str(self.shared))
        for key in ('RVIZ_IMAGE_TAG', 'IMAGE_TAG', 'EGO_RVIZ_IMAGE'):
            env.pop(key, None)
        ubuntu_record = self.shared / '.image.env'
        ubuntu_record.write_text('existing Ubuntu image record')
        for root in (self.shared, self.ego):
            result = subprocess.run(['/bin/bash', str(root / 'scripts/build_image_mac.sh')],
                                    env=env, text=True, capture_output=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
        calls = self.calls.read_text()
        self.assertIn('--platform linux/arm64', calls)
        self.assertIn('Dockerfile.macos', calls)
        self.assertIn('--build-arg BASE_IMAGE=ros-noetic-rviz:noetic-arm64-web', calls)
        self.assertIn('-t ego-swarm-rviz:noetic-arm64-web ', calls)
        self.assertTrue((self.shared / '.image.mac.env').exists())
        self.assertEqual(ubuntu_record.read_text(), 'existing Ubuntu image record')

    def test_checks_use_separate_containers(self):
        for ego in (False, True):
            result = self.launch(('--check',), ego=ego)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertRegex(result.stdout, r'--name rviz_\w+_check_\d+')
        self.assertFalse(self.calls.exists())

    def test_mac_parallel_rviz_names_and_no_open(self):
        for arguments in ((), ('--reloc',)):
            result = self.launch((*arguments, '--name', 'test-view.2', '--no-open'))
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('mac_rviz_test_view_2', result.stdout)
            self.assertIn('rviz_name', result.stdout)
        result = self.launch(('--no-open', '--no-map'), ego=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(self.calls.exists())

    def test_ego_symlink_resolves_repository(self):
        link = self.work / 'ego-rviz-mac'
        link.symlink_to(self.ego / 'scripts/run_mac.sh')
        env = dict(os.environ, EGO_RVIZ_SHARED=str(self.shared), BOARD_IP='192.168.31.32')
        result = subprocess.run(['/bin/bash', str(link), '--dry-run', '--no-map', '--ros-ip', '192.168.31.50'],
                                env=env, text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(str(self.ego).replace(' ', '\\ ') + '/home', result.stdout)

    def test_mac_layout_preserves_data_and_existing_personal_copy(self):
        spec = importlib.util.spec_from_file_location('layout', ROOT / 'tools/initialize_layout.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        source = ROOT / 'rviz/fastlio_mid360s.rviz'
        destination = self.work / 'private/home.rviz'
        module.initialize(str(source), str(destination))
        original = source.read_text()
        private = destination.read_text()
        self.assertEqual(original.split('\nWindow Geometry:')[0], private.split('\nWindow Geometry:')[0])
        self.assertIn('Decay Time: 1000', private)
        self.assertIn('Transport Hint: compressed', private)
        self.assertIn('Width: 1380', private)
        self.assertNotIn('QMainWindow State:', private)
        destination.write_text('my saved layout')
        module.initialize(str(source), str(destination))
        self.assertEqual(destination.read_text(), 'my saved layout')


if __name__ == '__main__':
    unittest.main()
