"""Optional board-local FAST-LIO timing. CSV rows share CLOCK_MONOTONIC in ns."""

import csv
import os
from contextlib import ExitStack
from pathlib import Path
import shutil
from tempfile import TemporaryDirectory
import threading
import time

import rospy
from mavros_msgs.msg import ExtendedState
from std_msgs.msg import String, UInt64MultiArray

# Transport order from FAST_LIO/src/laserMapping.cpp. Zero phase timestamps
# mean the frame did not reach that stage. Outcomes: 0 published, 1 first scan,
# 2 no undistorted points, 3 map initialization, 4 too few downsampled points.
FRAME_FIELDS = (
    'scan_id', 'pid', 'lidar_stamp_ns', 'odom_stamp_ns', 'lidar_enter_ns', 'lidar_ready_ns',
    'lidar_queue_depth', 'imu_count', 'input_points', 'downsampled_points', 'matched_points', 'outcome',
    'frame_start_ns', 'frame_start_cpu_ns', 'imu_done_ns', 'downsample_done_ns',
    'prelog_start_ns', 'prelog_done_ns', 'solve_start_ns', 'solve_start_cpu_ns',
    'solve_done_ns', 'solve_done_cpu_ns', 'odom_publish_ns',
    'map_start_ns', 'map_done_ns', 'frame_done_ns', 'frame_done_cpu_ns',
)
VM_FIELDS = ('nr_dirty', 'nr_writeback', 'nr_dirtied', 'nr_written')
TASK_FIELDS = ('State', 'voluntary_ctxt_switches', 'nonvoluntary_ctxt_switches')
IO_FIELDS = ('wchar', 'syscw', 'write_bytes')
OS_FIELDS = (
    'mono_ns', 'wall_ns', 'sample_end_ns', 'pid', 'task_runtime_ns', 'task_runqueue_ns', 'task_slices',
    'task_state', 'task_voluntary_switches', 'task_involuntary_switches', 'task_wchan',
    'fastlio_wchar', 'fastlio_write_syscalls', 'fastlio_write_bytes',
    'cpu_some_us', 'io_some_us', 'io_full_us', *VM_FIELDS,
    'device_sectors_read', 'device_writes', 'device_sectors_written', 'device_write_ms',
    'device_io_inflight', 'device_io_ms', 'bag_name', 'bag_size_bytes',
)
MAX_BYTES = 16 * 1024 * 1024
ARCHIVE_FILES = ('arrivals.csv', 'frames.csv', 'os.csv', 'metadata.txt')


def sync_directory(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def archive_checkpoint(source, destination):
    marker = '.checkpoint'
    expected = str(source) + '\n'
    if not destination.exists():
        with TemporaryDirectory(prefix='.timing.', dir=destination.parent) as temporary:
            stage = Path(temporary) / 'timing'
            stage.mkdir()
            for name in ARCHIVE_FILES:
                shutil.copyfile(source / name, stage / name)
            (stage / marker).write_text(expected)
            for path in stage.iterdir():
                with path.open('rb') as stream:
                    os.fsync(stream.fileno())
            sync_directory(stage)
            stage.rename(destination)
    if (destination / marker).read_text() != expected:
        raise FileExistsError('另一次运行已使用诊断归档目录 %s' % destination)
    sync_directory(destination.parent)


def read_text(path):
    try:
        return Path(path).read_text()
    except OSError:
        return ''


def read_fields(path, fields):
    values = {}
    for line in read_text(path).splitlines():
        parts = line.replace(':', ' ', 1).split(maxsplit=2)
        if len(parts) >= 2 and parts[0] in fields:
            values[parts[0]] = parts[1]
    return [values.get(key, '') for key in fields]


def pressure_total(path, *kinds):
    totals = {}
    for line in read_text(path).splitlines():
        kind, *values = line.split()
        for value in values:
            if value.startswith('total='):
                totals[kind] = value.partition('=')[2]
    return [totals.get(kind, '') for kind in kinds]


def block_stat(storage_path):
    dev = os.stat(storage_path).st_dev
    sysdev = Path('/sys/dev/block/%s:%s' % (os.major(dev), os.minor(dev))).resolve()
    if (sysdev / 'partition').exists():
        sysdev = sysdev.parent
    stat = sysdev / 'stat'
    return stat if stat.is_file() else None


class TimingLog:
    def __init__(self, directory, storage_path, log, archive_path=None):
        self.directory = Path(directory)
        self.log = log
        self.lock = threading.Lock()
        self.pid = None
        self.storage_path = Path(storage_path).resolve()
        self.block = block_stat(self.storage_path)
        self.bag_path = None
        self.bag_closed = False
        self.archive_path = Path(archive_path) if archive_path else None
        self.archived = False
        self.next_archive_attempt = 0.0
        self.was_armed = False
        self.fcu = (0.0, False)
        self.on_ground = (0.0, False)
        self.bytes = 0
        self.enabled = True
        self.files = ExitStack()
        try:
            self.arrivals = self.writer('arrivals.csv', ('mono_ns', 'wall_ns', 'topic', 'stamp_ns'))
            self.frames = self.writer('frames.csv', ('rx_mono_ns', *FRAME_FIELDS))
            self.os = self.writer('os.csv', OS_FIELDS)
            (self.directory / 'metadata.txt').write_text(
                'clock=CLOCK_MONOTONIC (ns)\n'
                'storage_path=%s\nblock_stat=%s\n'
                'sched_schedstats=%s\nvm_page_size=%s\nmax_csv_bytes=%s\n'
                % (storage_path, self.block or 'unavailable',
                   read_text('/proc/sys/kernel/sched_schedstats').strip() or 'unavailable',
                   os.sysconf('SC_PAGE_SIZE'), MAX_BYTES))
            self.sub = rospy.Subscriber('/fastlio/frame_timing', UInt64MultiArray, self.on_frame,
                                        queue_size=100, tcp_nodelay=True)
            self.bag_sub = rospy.Subscriber('/rc_record/bag_path', String, self.on_bag_path,
                                            queue_size=1, tcp_nodelay=True)
            self.ext_sub = rospy.Subscriber('/mavros/extended_state', ExtendedState, self.on_extended,
                                            queue_size=1, tcp_nodelay=True) if self.archive_path else None
        except OSError:
            self.files.close()
            raise

    def writer(self, filename, header):
        stream = self.files.enter_context(open(self.directory / filename, 'w', newline='', buffering=1))
        writer = csv.writer(stream)
        self.bytes += writer.writerow(header)
        return writer

    def write(self, writer, row):
        with self.lock:
            if not self.enabled:
                return
            try:
                self.bytes += writer.writerow(row)
                if self.bytes >= MAX_BYTES:
                    self.log('诊断 CSV 已到 16 MiB 上限，停止诊断写入')
                    self.enabled = False
            except OSError as exc:
                self.enabled = False
                self.log('诊断 CSV 写入停止：%s' % exc)

    def arrival(self, mono_ns, wall_ns, topic, stamp_ns):
        self.write(self.arrivals, (mono_ns, wall_ns, topic, stamp_ns))

    def on_frame(self, msg):
        mono_ns = time.monotonic_ns()
        if len(msg.data) == len(FRAME_FIELDS):
            self.pid = msg.data[1]
            self.write(self.frames, (mono_ns, *msg.data))

    def on_bag_path(self, msg):
        action, _, name = msg.data.partition(' ')
        path = Path(name)
        if action == 'OPEN' and path.parent.resolve() == self.storage_path:
            self.bag_path = path
            self.bag_closed = False
        elif self.bag_path == path:
            self.bag_path = None
            self.bag_closed = action == 'CLOSED'

    def on_state(self, msg):
        self.was_armed |= msg.connected and msg.armed
        self.fcu = (time.monotonic(), msg.connected and not msg.armed)

    def on_extended(self, msg):
        self.on_ground = (time.monotonic(), msg.landed_state == ExtendedState.LANDED_STATE_ON_GROUND)

    def checkpoint(self, now):
        if not self.archive_path or self.archived or now < self.next_archive_attempt:
            return
        received, disarmed = self.fcu
        extended_at, landed = self.on_ground
        if (not self.bag_closed or not self.was_armed or not disarmed or not landed
                or now - received > 2.0 or now - extended_at > 1.5):
            return
        try:
            archive_checkpoint(self.directory, self.archive_path)
        except OSError as exc:
            self.next_archive_attempt = now + 5.0
            self.log('落地诊断保存失败，正在等待重试：%s' % exc)
        else:
            self.archived = True
            self.log('FAST-LIO / CPU / 存储诊断已保存：%s（着地上锁且已关包）' % self.archive_path)

    def sample(self):
        if not self.enabled:
            return
        mono_ns, wall_ns = time.monotonic_ns(), time.time_ns()
        pid = self.pid
        sched = [''] * 3
        status = [''] * 3
        io = [''] * 3
        wchan = ''
        if pid:
            proc = '/proc/%d' % pid
            sched = (read_text(proc + '/schedstat').split() + sched)[:3]
            status = read_fields(proc + '/status', TASK_FIELDS)
            wchan = read_text(proc + '/wchan').strip()
            io = read_fields(proc + '/io', IO_FIELDS)
        cpu_pressure = pressure_total('/proc/pressure/cpu', 'some')
        io_pressure = pressure_total('/proc/pressure/io', 'some', 'full')
        vm = read_fields('/proc/vmstat', VM_FIELDS)
        block = read_text(self.block).split() if self.block else []
        block = [block[i] for i in (2, 4, 6, 7, 8, 9)] if len(block) >= 11 else [''] * 6
        bag_path = self.bag_path
        bag_size = ''
        if bag_path:
            try:
                bag_size = bag_path.stat().st_size
            except OSError:
                pass
        self.write(self.os, (mono_ns, wall_ns, time.monotonic_ns(), pid, *sched, *status, wchan, *io,
                             *cpu_pressure, *io_pressure, *vm, *block,
                             bag_path.name if bag_path else '', bag_size))

    def close(self):
        self.sub.unregister()
        self.bag_sub.unregister()
        if self.ext_sub:
            self.ext_sub.unregister()
        with self.lock:
            self.enabled = False
            self.files.close()
