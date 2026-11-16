#!/usr/bin/env python3
"""Order-page relay for a Windows PC that runs the robot inside WSL.

Some PCs have a firewall that blocks TCP between Windows and WSL, so a Windows
browser cannot open the order page served inside WSL. This relay listens on
http://127.0.0.1:8088 on Windows and forwards every request through one
long-running `wsl.exe` child process (its stdin/stdout pipes are not network
traffic) to warehouse_bot's web_stdio_proxy, which calls the ROS web server
inside WSL (127.0.0.2:8089, as started by tools/run_in_wsl.sh).

    python tools\\web_relay_windows.py --open-browser

run_warehouse.bat starts and stops it for you. Only the Python standard library is used.
On a plain Linux PC you do not need this: open http://localhost:8088 directly.
"""

import argparse
import base64
import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PID_FILE = os.path.join(tempfile.gettempdir(), 'warehouse_web_relay.pid')
MAX_BODY = 64 * 1024      # the order page only POSTs tiny JSON bodies
# web_stdio_proxy only uses the Python standard library, so it runs straight from this
# checkout - no ROS environment or built workspace needed on the WSL side.
PROXY_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         'warehouse_bot', 'warehouse_bot')


class WslPipe:
    """One persistent `wsl.exe ... web_stdio_proxy` process; requests are serialised."""

    def __init__(self, distro: str, user: str, host: str, port: int):
        # --cd takes the Windows path; -e runs python3 directly (no shell, so no quoting issues)
        self.cmd = ['wsl.exe', '-d', distro] + (['-u', user] if user else []) + [
            '--cd', PROXY_DIR, '-e', 'python3', '-u', 'web_stdio_proxy.py',
            '--host', host, '--port', str(port)]
        self.proc = None
        self.lock = threading.Lock()
        self.next_id = 0
        self.failed_at = 0.0

    def _start(self) -> None:
        flags = getattr(subprocess, 'CREATE_NO_WINDOW', 0)
        self.proc = subprocess.Popen(self.cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL, creationflags=flags)

    def _stop(self) -> None:
        if self.proc is not None:
            try:
                self.proc.kill()
            except OSError:
                pass
        self.proc = None

    def request(self, method: str, path: str, ctype: str, body: bytes):
        with self.lock:
            if self.proc is None or self.proc.poll() is not None:
                if time.monotonic() - self.failed_at < 3.0:
                    return self._error(503, 'WSL bridge is restarting')
                self._start()
            self.next_id += 1
            msg = {'id': self.next_id, 'method': method, 'path': path,
                   'headers': {'Content-Type': ctype} if ctype else {},
                   'body': base64.b64encode(body).decode('ascii')}
            try:
                self.proc.stdin.write(json.dumps(msg).encode() + b'\n')
                self.proc.stdin.flush()
                line = self.proc.stdout.readline()
                if not line:
                    raise OSError('WSL bridge exited')
                resp = json.loads(line)
                return (int(resp['status']), resp['headers'].get('Content-Type', 'text/plain'),
                        base64.b64decode(resp['body']))
            except (OSError, ValueError, KeyError) as exc:
                self.failed_at = time.monotonic()
                self._stop()
                return self._error(503, f'WSL bridge not ready ({exc})')

    @staticmethod
    def _error(code: int, message: str):
        return code, 'application/json', json.dumps({'error': message}).encode()


def make_handler(pipe: WslPipe):

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _relay(self, method: str) -> None:
            try:
                length = int(self.headers.get('Content-Length') or 0)
            except ValueError:
                length = -1
            if length < 0 or length > MAX_BODY:
                self.send_error(413, 'Request body too large')
                return
            body = self.rfile.read(length) if length else b''
            code, ctype, data = pipe.request(method, self.path, self.headers.get('Content-Type', ''),
                                             body)
            try:
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)
            except (ConnectionError, OSError):
                pass        # the browser gave up waiting (e.g. while ROS was starting)

        def do_GET(self):
            self._relay('GET')

        def do_POST(self):
            self._relay('POST')

    return Handler


def stop_previous() -> None:
    """Stop a relay left over from an earlier run - but only if the PID in the PID file
    still belongs to this relay (Windows reuses PIDs, so never kill blindly)."""
    try:
        with open(PID_FILE) as f:
            pid = int(f.read().strip())
    except (OSError, ValueError):
        return
    if pid != os.getpid():
        cmdline = ''
        try:
            out = subprocess.run(
                ['powershell', '-NoProfile', '-Command',
                 f'(Get-CimInstance Win32_Process -Filter "ProcessId={pid}").CommandLine'],
                capture_output=True, text=True, timeout=20,
                creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            cmdline = out.stdout
        except (OSError, subprocess.SubprocessError):
            pass
        if 'web_relay_windows' in cmdline:
            try:
                os.kill(pid, signal.SIGTERM)
                print(f'[order page] stopped the earlier relay (PID {pid})', flush=True)
            except OSError:
                pass
    try:
        os.remove(PID_FILE)
    except OSError:
        pass


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--stop', action='store_true',
                    help='stop a running relay (used by run_warehouse.bat) and exit')
    ap.add_argument('--port', type=int, default=8088, help='Windows port (default 8088)')
    ap.add_argument('--distro', default=os.environ.get('WAREHOUSE_WSL_DISTRO', 'Ubuntu-22.04'),
                    help='WSL distribution (default Ubuntu-22.04)')
    ap.add_argument('--user', default='', help='WSL user (default: the distribution default user)')
    ap.add_argument('--wsl-host', default='127.0.0.2', help='web_server address inside WSL')
    ap.add_argument('--wsl-port', type=int, default=8089, help='web_server port inside WSL')
    ap.add_argument('--open-browser', action='store_true')
    args = ap.parse_args()

    stop_previous()
    if args.stop:
        return
    pipe = WslPipe(args.distro, args.user, args.wsl_host, args.wsl_port)
    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(pipe))
    except OSError as exc:
        print(f'[order page] cannot use port {args.port} on Windows: {exc}', file=sys.stderr)
        sys.exit(1)
    server.daemon_threads = True
    with open(PID_FILE, 'w') as f:
        f.write(str(os.getpid()))
    url = f'http://localhost:{args.port}'
    print(f'[order page] {url}  (relayed into WSL {args.distro})', flush=True)
    if args.open_browser:
        threading.Timer(1.0, webbrowser.open, args=(url,)).start()
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        pipe._stop()
        try:
            os.remove(PID_FILE)
        except OSError:
            pass


if __name__ == '__main__':
    main()
