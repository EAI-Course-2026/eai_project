"""Local-only browser control panel for the SCS215 SO101 arm."""
import argparse
import base64
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from http.cookies import SimpleCookie
import json
from pathlib import Path
import sys
import webbrowser

from eai_robot.config import ROOT, load_config
from eai_robot.applications.tools.arm import DEFAULT_CALIBRATION
from eai_robot.applications.tools.demo import DEFAULT_HOME
from eai_robot.applications.desktop.service import ArmDeskService

ASSETS = Path(__file__).with_name('assets')
MIMES = {'/': ('index.html', 'text/html; charset=utf-8'),
         '/app.css': ('app.css', 'text/css; charset=utf-8'),
         '/app.js': ('app.js', 'text/javascript; charset=utf-8')}
MIMES['/agent.js'] = ('agent.js', 'text/javascript; charset=utf-8')
MIMES['/agent.css'] = ('agent.css', 'text/css; charset=utf-8')
MIMES['/agent-settings.js'] = ('agent-settings.js', 'text/javascript; charset=utf-8')


def discover_ports():
    """Enumerate OS ports only; never open devices or probe motors."""
    from serial.tools import list_ports
    return [{'device': p.device, 'description': p.description or '',
             'manufacturer': p.manufacturer or ''}
            for p in sorted(list_ports.comports(), key=lambda p: p.device)]


def make_handler(service, initial_port, agent=None):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            pass

        def _record(self):
            from .agent_config import AgentHub
            if not isinstance(agent, AgentHub):
                return None
            if hasattr(self, '_record_cache'):
                return self._record_cache
            cookies = SimpleCookie()
            cookie_name = f'eai_agent_{self.server.server_port}'
            try:
                cookies.load(self.headers.get('Cookie', ''))
                token = cookies[cookie_name].value if cookie_name in cookies else None
            except Exception:
                token = None
            selected, record = agent.get(token)
            if selected != token:
                self._session_cookie = f'{cookie_name}={selected}; HttpOnly; SameSite=Strict; Path=/api/agent'
            self._record_cache = record
            return record

        def _send(self, code, body, mime='application/json; charset=utf-8'):
            self.send_response(code)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            if hasattr(self, '_session_cookie'):
                self.send_header('Set-Cookie', self._session_cookie)
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; connect-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'; base-uri 'none'; frame-ancestors 'none'")
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path.startswith('/api/agent'):
                if self.headers.get('Host') not in {f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'}:
                    return self._send(403, b'{"error":"forbidden"}')
                try:
                    record = self._record()
                except ValueError:
                    return self._send(429, b'{"error":"session_limit"}')
                runtime = record.agent if record else agent
                if path == '/api/agent/state':
                    data = runtime.snapshot() if runtime else {'enabled': False, 'speech_enabled': False, 'messages': [], 'tools': []}
                    data['api_settings'] = record.public_settings() if record else {}
                    return self._send(200, json.dumps(data, ensure_ascii=False).encode())
                if path == '/api/agent/catalog':
                    from .agent_config import catalog
                    return self._send(200, json.dumps(catalog(), ensure_ascii=False).encode())
                if path == '/api/agent/frame.jpg':
                    frame, _ = agent.hardware_frame() if hasattr(agent, 'hardware_frame') else (None, 0)
                    if frame is not None:
                        return self._send(200, frame, 'image/jpeg')
                    return self._send(404, b'{"error":"frame_unavailable"}')
                if path.startswith('/api/agent/audio/') and runtime:
                    try:
                        parts = path.split('/')
                        audio = runtime.audio(int(parts[-1])) if len(parts) == 6 and parts[-2] == runtime.runtime_id else None
                    except ValueError:
                        audio = None
                    if audio is not None:
                        return self._send(200, audio, 'audio/wav')
                return self._send(404, b'{"error":"not found"}')
            if path == '/api/ports':
                try:
                    return self._send(200, json.dumps({'ports': discover_ports()}, ensure_ascii=False).encode())
                except Exception as exc:
                    return self._send(503, json.dumps({'error': f'串口扫描失败：{exc}'}, ensure_ascii=False).encode())
            if path == '/api/state':
                data = service.snapshot()
                data['default_port'] = initial_port
                return self._send(200, json.dumps(data, ensure_ascii=False).encode())
            if path in MIMES:
                filename, mime = MIMES[path]
                return self._send(200, (ASSETS / filename).read_bytes(), mime)
            self._send(404, b'{"error":"not found"}')

        def do_POST(self):
            if self.path not in {'/api/action', '/api/agent/turn', '/api/agent/stop',
                                 '/api/agent/config', '/api/agent/clear',
                                 '/api/agent/preview', '/api/agent/release'}:
                return self._send(404, b'{"error":"not found"}')
            origin = self.headers.get('Origin')
            expected = f'http://{self.headers.get("Host", "")}'
            if self.headers.get('X-Arm-Desk') != '1' or origin and origin != expected:
                return self._send(403, b'{"error":"forbidden"}')
            if self.path.startswith('/api/agent') and self.headers.get('Host') not in {f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'}:
                return self._send(403, b'{"error":"forbidden"}')
            try:
                length = int(self.headers.get('Content-Length', '0'))
                maximum = 3 * 1024 * 1024 if self.path == '/api/agent/turn' else 65536 if self.path == '/api/agent/config' else 16384
                if not 0 < length <= maximum:
                    raise ValueError('请求大小无效')
                request = json.loads(self.rfile.read(length))
                if not isinstance(request, dict):
                    raise ValueError('请求格式无效')
                if self.path.startswith('/api/agent'):
                    record = self._record()
                    runtime = record.agent if record else agent
                    if runtime is None:
                        raise ValueError('agent_not_configured')
                    if self.path.endswith('/config') and record:
                        result = record.configure(request)
                        return self._send(200, json.dumps({'api_settings': result}, ensure_ascii=False).encode())
                    elif self.path.endswith('/clear') and record:
                        record.clear()
                    elif self.path.endswith('/preview'):
                        if set(request) != {'enabled'} or type(request['enabled']) is not bool:
                            raise ValueError('invalid_input')
                        result = agent.preview(request['enabled'])
                        return self._send(200, json.dumps(result, ensure_ascii=False).encode())
                    elif self.path.endswith('/release'):
                        if set(request) != {'confirmation'}:
                            raise ValueError('invalid_input')
                        result = agent.release(request.get('confirmation'))
                        return self._send(200, json.dumps(result, ensure_ascii=False).encode())
                    elif self.path.endswith('/stop'):
                        runtime.stop()
                    else:
                        if set(request) - {'text', 'audio', 'speak', 'generation', 'runtime_id'} or type(request.get('generation')) is not int:
                            raise ValueError('invalid_input')
                        if 'audio' in request and not isinstance(request['audio'], str):
                            raise ValueError('invalid_input')
                        audio = base64.b64decode(request['audio'], validate=True) if 'audio' in request else None
                        arguments = dict(text=request.get('text'), audio=audio, speak=request.get('speak', True), expected_generation=request['generation'])
                        if record:
                            record.submit(request.get('runtime_id'), **arguments)
                        else:
                            runtime.submit(**arguments)
                    return self._send(202, b'{"accepted":true}')
                if request.get('action') == 'stop' and agent is not None:
                    agent.stop()
                if (hasattr(agent, 'hardware_snapshot') and agent.hardware_snapshot()
                        and request.get('action') != 'stop'):
                    raise ValueError('hardware_agent_mode_enabled')
                service.submit(request.get('action'), request.get('payload'))
                return self._send(202, b'{"accepted":true}')
            except (ValueError, RuntimeError) as exc:
                self._send(400, json.dumps({'error': str(exc)}, ensure_ascii=False).encode())
            except Exception as exc:
                if self.path.startswith('/api/agent'):
                    return self._send(500, b'{"error":"agent_service_error"}')
                self._send(500, json.dumps({'error': f'{type(exc).__name__}: {exc}'}, ensure_ascii=False).encode())
    return Handler


def main(argv=None):
    parser = argparse.ArgumentParser(description='本机浏览器六关节 SCS215 上位机')
    parser.add_argument('--http-port', type=int, default=0, help='本机浏览器端口；0=自动分配')
    parser.add_argument('--calibration', type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument('--home-file', type=Path, default=DEFAULT_HOME)
    parser.add_argument('--device-port', default=None)
    parser.add_argument('--no-open', action='store_true', help='只启动服务，不自动打开浏览器')
    parser.add_argument('--agent-config', type=Path, help='Optional chat API TOML seed; default uses web configuration')
    parser.add_argument('--agent-env-file', type=Path)
    parser.add_argument('--speech-config', type=Path, help='Independent cloud ASR/TTS TOML')
    parser.add_argument('--speech-env-file', type=Path)
    parser.add_argument('--visual-hardware-profile', type=Path,
                        help='Enable the browser Agent hardware backend with this validated profile')
    parser.add_argument('--camera-index', type=int)
    parser.add_argument('--shared-frame', type=Path,
                        help='Read fresh frames from a separate persistent preview process')
    parser.add_argument('--hardware-max-runtime', type=float, default=30.0)
    parser.add_argument('--enable-visual-motion', action='store_true')
    parser.add_argument('--acknowledge')
    args = parser.parse_args(argv)
    initial_port = args.device_port if args.device_port is not None else load_config()['serial']['port']
    service = ArmDeskService(args.calibration, args.home_file)
    agent = None
    try:
        from .agent_config import AgentHub, seed_from_files
        hardware_skills = None
        hardware_requested = any((args.visual_hardware_profile is not None,
                                  args.camera_index is not None,
                                  args.shared_frame is not None,
                                  args.enable_visual_motion,
                                  args.acknowledge is not None))
        if hardware_requested:
            from eai_robot.applications.agent.hardware_visual import (
                ArucoCenterHardwareBackend, HARDWARE_ACKNOWLEDGEMENT,
                HardwareVisualSkills,
            )
            if (not args.enable_visual_motion or args.visual_hardware_profile is None
                    or args.device_port is None or args.camera_index is None
                    or args.acknowledge != HARDWARE_ACKNOWLEDGEMENT):
                parser.error('网页真机 Agent 需要 profile、device-port、camera-index、'
                             '--enable-visual-motion 和正确的 --acknowledge')
            backend = ArucoCenterHardwareBackend(
                args.visual_hardware_profile, port=args.device_port,
                camera_index=args.camera_index, max_runtime_s=args.hardware_max_runtime,
                shared_frame=args.shared_frame)
            hardware_skills = HardwareVisualSkills(backend)
        def stop_connected_robot():
            if hardware_skills:
                hardware_skills.backend.stop()
            if service.snapshot()['connected']:
                service.submit('stop')
        if not args.agent_config and args.agent_env_file:
            parser.error('--agent-env-file requires --agent-config')
        if not args.speech_config and args.speech_env_file:
            parser.error('--speech-env-file requires --speech-config')
        initial = seed_from_files(args.agent_config, args.agent_env_file, args.speech_config, args.speech_env_file)
        agent = AgentHub(on_stop=stop_connected_robot, initial=initial,
                         hardware_skills=hardware_skills)
        server = ThreadingHTTPServer(('127.0.0.1', args.http_port), make_handler(service, initial_port, agent))
        server.daemon_threads = True
        url = f'http://127.0.0.1:{server.server_port}/'
        print(f'上位机已启动：{url}', flush=True)
        if hardware_skills:
            source = '共享相机帧' if args.shared_frame else '直接相机'
            print(f'网页 Agent 真机模式已启用（{source}）；网页预览默认关闭，按需开启。', flush=True)
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
        try:
            if agent:
                agent.close()
        finally:
            service.close()
    return 0
