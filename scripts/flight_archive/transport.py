"""Bounded log transfers over an already open pymavlink connection."""
from pathlib import Path
import os
import time

BLOCK = 90
WINDOW = BLOCK * 600


class LogTransfer:
    def __init__(self, link, check=lambda: None):
        self.link = link
        self.check = check
        self.system = link.target_system
        self.component = link.target_component

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.link.mav.log_request_end_send(self.system, self.component)

    def reply(self, kind):
        self.check()
        message = self.link.recv_match(type=kind, blocking=True, timeout=0.25)
        if message and (message.get_srcSystem(), message.get_srcComponent()) == (self.system, self.component):
            return message
        return None

    def list_logs(self):
        self.check()
        self.link.mav.log_request_list_send(self.system, self.component, 0, 0xffff)
        entries = {}
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            message = self.reply("LOG_ENTRY")
            if message is None:
                continue
            if message.num_logs == 0:
                return []
            entries[message.id] = message
            if len(entries) == message.num_logs:
                return sorted(entries.values(), key=lambda entry: entry.id)
        raise TimeoutError("PX4 日志清单响应不完整")

    def read(self, entry, start, size):
        """Receive a bounded byte range, requesting missing aligned ranges again."""
        expected = min(size, entry.size - start)
        if expected <= 0:
            raise ValueError("空的 ULog 字节区间")
        end = start + expected
        blocks = {}
        deadline = time.monotonic() + 35
        offset = start
        while offset < end and time.monotonic() < deadline:
            self.check()
            self.link.mav.log_request_data_send(
                self.system, self.component, entry.id, offset, end - offset
            )
            idle = time.monotonic() + 1.2
            while time.monotonic() < min(idle, deadline):
                message = self.reply("LOG_DATA")
                if message is None or message.id != entry.id:
                    continue
                pos = message.ofs
                if not start <= pos < end or (pos - start) % BLOCK != 0:
                    continue
                needed = min(BLOCK, end - pos)
                if message.count < needed or pos in blocks:
                    continue
                blocks[pos] = bytes(message.data[:needed])
                while offset < end and offset in blocks:
                    offset += BLOCK
                if offset >= end:
                    break
                idle = time.monotonic() + 1.2
        if offset < end:
            raise TimeoutError("ULog {} 字节 {} 传输超时".format(entry.id, offset))
        return b"".join(blocks[pos] for pos in range(start, end, BLOCK))

    def fetch(self, entry, part_path):
        path = Path(part_path)
        with path.open("wb") as stream:
            for start in range(0, entry.size, WINDOW):
                stream.write(self.read(entry, start, WINDOW))
            stream.flush()
            os.fsync(stream.fileno())
        if path.stat().st_size != entry.size:
            raise ValueError("ULog 下载字节数与 PX4 清单不一致")
