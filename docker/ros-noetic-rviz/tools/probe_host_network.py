#!/usr/bin/env python3
"""Probe host↔container TCP through the advertised host IP; never contact ROS."""
import os
import selectors
import socket
import socketserver
import subprocess
import sys
import threading
import time

CONTAINER = '''
import socket, sys
with socket.create_connection((sys.argv[1], int(sys.argv[2])), timeout=5) as s:
    s.sendall(b'probe')
    assert s.recv(64) == b'host-ok', 'container to host failed'
with socket.socket() as s:
    s.bind(('0.0.0.0', 0)); s.listen(1); s.settimeout(10)
    print(s.getsockname()[1], flush=True)
    c, _ = s.accept()
    with c:
        assert c.recv(64) == b'probe'
        c.sendall(b'container-ok')
'''


class Handler(socketserver.BaseRequestHandler):
    def handle(self):
        self.request.settimeout(5)
        if self.request.recv(64) == b'probe':
            self.request.sendall(b'host-ok')


def probe(image, host_ip):
    name = 'rviz-network-check-%d' % os.getpid()
    proc = None
    with socketserver.TCPServer(('0.0.0.0', 0), Handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            proc = subprocess.Popen(['docker', 'run', '--rm', '--name', name, '--network', 'host',
                                     image, 'python3', '-u', '-c', CONTAINER, host_ip,
                                     str(server.server_address[1])], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            with selectors.DefaultSelector() as selector:
                selector.register(proc.stdout, selectors.EVENT_READ)
                if not selector.select(timeout=20):
                    raise RuntimeError('container did not reach host TCP server within 20s')
            line = proc.stdout.readline().strip()
            if not line.isdigit():
                _, error = proc.communicate(timeout=5)
                raise RuntimeError('container to host failed: ' + error.decode(errors='replace')[-800:])
            # Desktop may expose a just-created listener a little after the
            # container has bound it. Retry refusals before reporting failure.
            deadline = time.monotonic() + 5
            while True:
                try:
                    sock = socket.create_connection((host_ip, int(line)), timeout=1)
                    break
                except OSError as exc:
                    if time.monotonic() >= deadline:
                        raise RuntimeError('host to container TCP listener unreachable: %s' % exc) from exc
                    time.sleep(.1)
            with sock:
                sock.settimeout(5)
                sock.sendall(b'probe')
                if sock.recv(64) != b'container-ok':
                    raise RuntimeError('host to container response invalid')
            _, error = proc.communicate(timeout=5)
            if proc.returncode:
                raise RuntimeError(error.decode(errors='replace'))
            print('HOST_NETWORK_OK %s (host↔container TCP; board callback still needs live verification)' % host_ip)
        finally:
            if proc and proc.poll() is None:
                subprocess.run(['docker', 'rm', '-f', name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
                proc.communicate(timeout=10)
            server.shutdown()


if __name__ == '__main__':
    try:
        probe(*sys.argv[1:])
    except Exception as exc:
        sys.exit('HOST_NETWORK_FAILED: %s' % exc)
