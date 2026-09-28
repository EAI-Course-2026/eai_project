"""Local-only browser control panel for the SCS215 SO101 arm."""
import argparse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
import webbrowser

from eai_robot.config import ROOT, load_config
from .cli import DEFAULT_CALIBRATION
from .demo import DEFAULT_HOME
from .desk_service import ArmDeskService

ASSETS = Path(__file__).with_name('desk_assets')
MIMES = {'/': ('index.html', 'text/html; charset=utf-8'),
         '/app.css': ('app.css', 'text/css; charset=utf-8'),
         '/app.js': ('app.js', 'text/javascript; charset=utf-8')}


def make_handler(service, initial_port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            pass

        def _send(self, code, body, mime='application/json; charset=utf-8'):
            self.send_response(code)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; connect-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'; base-uri 'none'; frame-ancestors 'none'")
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/api/state':
                data = service.snapshot()
                data['default_port'] = initial_port
                return self._send(200, json.dumps(data, ensure_ascii=False).encode())
            if path in MIMES:
                filename, mime = MIMES[path]
                return self._send(200, (ASSETS / filename).read_bytes(), mime)
            self._send(404, b'{"error":"not found"}')

        def do_POST(self):
            if self.path != '/api/action':
                return self._send(404, b'{"error":"not found"}')
            origin = self.headers.get('Origin')
            expected = f'http://{self.headers.get("Host", "")}'
            if self.headers.get('X-Arm-Desk') != '1' or origin and origin != expected:
                return self._send(403, b'{"error":"forbidden"}')
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 16384:
                    raise ValueError('请求大小无效')
                request = json.loads(self.rfile.read(length))
                if not isinstance(request, dict):
                    raise ValueError('请求格式无效')
                service.submit(request.get('action'), request.get('payload'))
                return self._send(202, b'{"accepted":true}')
            except (ValueError, RuntimeError) as exc:
                self._send(400, json.dumps({'error': str(exc)}, ensure_ascii=False).encode())
            except Exception as exc:
                self._send(500, json.dumps({'error': f'{type(exc).__name__}: {exc}'}, ensure_ascii=False).encode())
    return Handler


def main(argv=None):
    parser = argparse.ArgumentParser(description='本机浏览器六关节 SCS215 上位机')
    parser.add_argument('--http-port', type=int, default=0, help='本机浏览器端口；0=自动分配')
    parser.add_argument('--calibration', type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument('--home-file', type=Path, default=DEFAULT_HOME)
    parser.add_argument('--device-port', default=None)
    parser.add_argument('--no-open', action='store_true', help='只启动服务，不自动打开浏览器')
    args = parser.parse_args(argv)
    initial_port = args.device_port if args.device_port is not None else load_config()['serial']['port']
    service = ArmDeskService(args.calibration, args.home_file)
    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.http_port), make_handler(service, initial_port))
        server.daemon_threads = True
        url = f'http://127.0.0.1:{server.server_port}/'
        print(f'上位机已启动：{url}', flush=True)
        if not args.no_open:
            webbrowser.open(url)
        try:
            server.serve_forever(poll_interval=.2)
        except KeyboardInterrupt:
            print('正在关闭上位机并释放扭矩…', flush=True)
        finally:
            server.shutdown()
            server.server_close()
    finally:
        service.close()
    return 0
