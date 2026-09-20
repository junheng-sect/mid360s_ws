"""六位序号 bag 路径分配（崩溃安全）。

参照板端 orange 工作空间 vz_step_control/scripts/sequence_allocator.py 的语义：
状态文件保存「最后分配的序号」，所以崩溃最多留下空号，绝不会复用旧编号；
锁文件独立存在，因为状态文件是原子替换（os.replace）的。
"""

import fcntl
import glob
import os

SEQUENCE_WIDTH = 6


class SequenceAllocator(object):
    def __init__(self, bag_dir, prefix, width=SEQUENCE_WIDTH):
        self.bag_dir = os.path.abspath(bag_dir)
        self.prefix = prefix
        self.width = width
        self.state_path = os.path.join(self.bag_dir,
                                       '.{}_sequence'.format(prefix))
        self.lock_path = self.state_path + '.lock'

    def allocate(self):
        """保留并返回下一个唯一的 <prefix>_%06d.bag 路径。"""
        os.makedirs(self.bag_dir, exist_ok=True)
        with open(self.lock_path, 'a+') as lock_file:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
            last = self._read_last_sequence()
            sequence = max(last, self._highest_existing_sequence()) + 1
            candidate = self._candidate_path(sequence)
            while os.path.exists(candidate) or os.path.exists(candidate + '.active'):
                sequence += 1
                candidate = self._candidate_path(sequence)
            self._write_last_sequence(sequence)
            return candidate

    def _candidate_path(self, sequence):
        return os.path.join(
            self.bag_dir,
            '{}_{:0{}d}.bag'.format(self.prefix, sequence, self.width),
        )

    def _read_last_sequence(self):
        if not os.path.exists(self.state_path):
            return 0
        with open(self.state_path, 'r') as state_file:
            value = state_file.read().strip()
        try:
            sequence = int(value)
        except ValueError:
            raise ValueError(
                'invalid bag sequence state in {}'.format(self.state_path))
        if sequence < 0:
            raise ValueError(
                'negative bag sequence state in {}'.format(self.state_path))
        return sequence

    def _highest_existing_sequence(self):
        highest = 0
        prefix = '{}_'.format(self.prefix)
        for path in glob.glob(os.path.join(self.bag_dir, '{}*.bag*'.format(prefix))):
            name = os.path.basename(path)
            if name.endswith('.bag.active'):
                stem = name[:-len('.bag.active')]
            elif name.endswith('.bag'):
                stem = name[:-len('.bag')]
            else:
                continue
            value = stem[len(prefix):]
            if value.isdigit():
                highest = max(highest, int(value))
        return highest

    def _write_last_sequence(self, sequence):
        temp_path = '{}.tmp.{}'.format(self.state_path, os.getpid())
        try:
            with open(temp_path, 'w') as state_file:
                state_file.write('{}\n'.format(sequence))
                state_file.flush()
                os.fsync(state_file.fileno())
            os.replace(temp_path, self.state_path)
            directory_fd = os.open(self.bag_dir, os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)
