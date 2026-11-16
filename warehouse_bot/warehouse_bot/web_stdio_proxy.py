#!/usr/bin/env python3
"""Forward HTTP requests that arrive as JSON lines on stdin to the order web server.

Used by tools/web_relay_windows.py on a Windows + WSL PC whose firewall blocks
TCP between Windows and WSL: the relay starts this script with `wsl.exe` and
talks to it over the process's stdin/stdout pipes, which the firewall cannot
see. Only the Python standard library is needed.

    request  line: {"id": 1, "method": "GET", "path": "/api/status",
                    "headers": {"Content-Type": "..."}, "body": "<base64>"}
    response line: {"id": 1, "status": 200, "headers": {"Content-Type": "..."},
                    "body": "<base64>"}
"""

import argparse
import base64
import http.client
import json
import sys


def b64(data: bytes) -> str:
    return base64.b64encode(data).decode('ascii')


def forward(req: dict, host: str, port: int, timeout: float) -> dict:
    rid = req.get('id')
    try:
        conn = http.client.HTTPConnection(host, port, timeout=timeout)
        body = base64.b64decode(req.get('body') or '')
        headers = {k: v for k, v in (req.get('headers') or {}).items()
                   if k.lower() == 'content-type'}
        conn.request(req.get('method', 'GET'), req.get('path', '/'), body=body or None,
                     headers=headers)
        r = conn.getresponse()
        data = r.read()
        conn.close()
        return {'id': rid, 'status': r.status,
                'headers': {'Content-Type': r.getheader('Content-Type',
                                                        'application/octet-stream')},
                'body': b64(data)}
    except OSError as exc:
        err = json.dumps({'error': f'order server not reachable inside WSL ({exc})'}).encode()
        return {'id': rid, 'status': 502, 'headers': {'Content-Type': 'application/json'},
                'body': b64(err)}


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.2')
    ap.add_argument('--port', type=int, default=8089)
    ap.add_argument('--timeout', type=float, default=15.0)
    args = ap.parse_args(argv)
    out = sys.stdout.buffer
    for line in sys.stdin.buffer:
        try:
            req = json.loads(line)
        except ValueError:
            continue
        out.write(json.dumps(forward(req, args.host, args.port, args.timeout)).encode() + b'\n')
        out.flush()


if __name__ == '__main__':
    main()
