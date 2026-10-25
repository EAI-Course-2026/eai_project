"""Web API configuration privacy, browser isolation and atomic replacement."""
from http.cookiejar import CookieJar
from http.server import ThreadingHTTPServer
import json
from threading import Thread
import time
from types import SimpleNamespace
import unittest
from urllib.error import HTTPError
from urllib.request import build_opener, HTTPCookieProcessor, Request
from unittest.mock import patch

from eai_robot.applications.desktop.agent_config import AgentHub, AgentRecord
from eai_robot.applications.desktop.server import make_handler


def settings(url='https://example.invalid/v1', key='PRIVATE_SESSION_KEY'):
    return {'chat': {'provider': 'custom', 'adapter': 'chat_completions', 'execution': 'managed_api',
                     'base_url': url, 'model': 'fixture', 'max_tokens': 128, 'api_key': key}}


class WebConfigurationTests(unittest.TestCase):
    def test_hardware_state_preview_frame_and_confirmed_release_are_web_accessible(self):
        class Backend:
            def __init__(self):
                self.previewing = False
                self.released = 0
            def snapshot(self):
                return {'mode':'aruco_center_hardware','connected':False,
                        'tracking':'idle','hardware_sent':False,
                        'motion_command_count':0,'motion_confirmed_count':0,
                        'torque_enabled':False,'requires_operator_release':False,
                        'error_code':None,'hold_reason':None,
                        'camera_preview':self.previewing,'frame_sequence':1,
                        'frame_source':'shared'}
            def frame(self):
                return b'fake-jpeg', 1
            def start_preview(self):
                self.previewing=True;return self.snapshot()
            def stop_preview(self, *args, **kwargs):
                self.previewing=False;return self.snapshot()
            def release(self):
                self.released+=1;return self.snapshot()
        backend=Backend();skills=SimpleNamespace(backend=backend,release=backend.release)
        hub=AgentHub(hardware_skills=skills);self.addCleanup(hub.close)
        desk=SimpleNamespace(snapshot=lambda:{},submit=lambda *args:None)
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(desk,'FAKE',hub))
        Thread(target=server.serve_forever,daemon=True).start()
        self.addCleanup(server.server_close);self.addCleanup(server.shutdown)
        opener=build_opener(HTTPCookieProcessor(CookieJar()))
        base=f'http://127.0.0.1:{server.server_port}'
        state=json.load(opener.open(base+'/api/agent/state'))
        self.assertEqual(state['mode'],'aruco_center_hardware')
        self.assertEqual(state['hardware']['frame_source'],'shared')
        self.assertEqual(state['hardware']['motion_confirmed_count'],0)
        request=Request(base+'/api/agent/preview',data=b'{"enabled":true}',
                        headers={'X-Arm-Desk':'1','Content-Type':'application/json'})
        self.assertTrue(json.load(opener.open(request))['camera_preview'])
        self.assertEqual(opener.open(base+'/api/agent/frame.jpg').read(),b'fake-jpeg')
        bad=Request(base+'/api/agent/release',data=b'{"confirmation":"wrong"}',
                    headers={'X-Arm-Desk':'1','Content-Type':'application/json'})
        with self.assertRaises(HTTPError):opener.open(bad)
        good=Request(base+'/api/agent/release',data=b'{"confirmation":"RELEASE TORQUE"}',
                     headers={'X-Arm-Desk':'1','Content-Type':'application/json'})
        self.assertEqual(opener.open(good).status,200)
        self.assertEqual(backend.released,1)

    def test_configuration_is_offline_secret_never_returned_and_invalid_update_is_atomic(self):
        record=AgentRecord();self.addCleanup(record.close)
        with patch('httpx.Client.send',side_effect=AssertionError('Configuration must not request API')):
            public=record.configure(settings())
        self.assertTrue(public['chat']['key_configured'])
        self.assertNotIn('PRIVATE_SESSION_KEY',json.dumps(public))
        self.assertNotIn('api_key',public['chat'])
        runtime=record.agent.runtime_id
        record.configure(settings(key=''))
        self.assertNotEqual(record.agent.runtime_id,runtime)
        runtime=record.agent.runtime_id
        with self.assertRaisesRegex(ValueError,'credential_required_for_new_endpoint'):
            record.configure(settings('https://other.invalid/v1',key=''))
        self.assertEqual(record.agent.runtime_id,runtime)
        with self.assertRaises(ValueError) as caught:
            record.configure(settings('https://user:PRIVATE_URL_SECRET@example.invalid/v1'))
        self.assertNotIn('PRIVATE_URL_SECRET',str(caught.exception))
        self.assertEqual(record.agent.runtime_id,runtime)
        record.clear();self.assertEqual(record.public_settings(),{})
        self.assertFalse(record.agent.snapshot()['enabled'])

    def test_two_browser_sessions_cannot_share_configuration_or_history(self):
        hub=AgentHub();self.addCleanup(hub.close)
        first,a=hub.get();second,b=hub.get()
        self.assertNotEqual(first,second)
        a.configure(settings())
        self.assertEqual(b.public_settings(),{})
        self.assertIs(hub.get(first)[1],a)
        self.assertNotEqual(a.agent.runtime_id,b.agent.runtime_id)
        a.clear();self.assertEqual(a.public_settings(),{})

    def test_idle_expiration_and_bound_do_not_leave_unbounded_credentials(self):
        hub=AgentHub();self.addCleanup(hub.close)
        token,record=hub.get();record.configure(settings())
        record.touched=time.monotonic()-3601
        replacement,new_record=hub.get(token)
        self.assertNotEqual(replacement,token)
        self.assertEqual(record.public_settings(),{})
        for _ in range(15):hub.get()
        with self.assertRaisesRegex(ValueError,'session_limit'):hub.get()

    def test_http_settings_are_cookie_scoped_and_stale_runtime_cannot_submit(self):
        hub=AgentHub();self.addCleanup(hub.close)
        desk=SimpleNamespace(snapshot=lambda:{},submit=lambda *args:None)
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(desk,'FAKE',hub))
        thread=Thread(target=server.serve_forever,daemon=True);thread.start()
        self.addCleanup(server.server_close);self.addCleanup(server.shutdown)
        base=f'http://127.0.0.1:{server.server_port}'
        a=build_opener(HTTPCookieProcessor(CookieJar()));b=build_opener(HTTPCookieProcessor(CookieJar()))
        response=a.open(base+'/api/agent/state')
        cookie=response.headers['Set-Cookie'];old=json.load(response)
        self.assertIn('HttpOnly',cookie);self.assertIn('SameSite=Strict',cookie)
        body=json.dumps(settings()).encode()
        request=Request(base+'/api/agent/config',data=body,headers={'X-Arm-Desk':'1','Content-Type':'application/json'})
        public=json.load(a.open(request))
        self.assertNotIn('PRIVATE_SESSION_KEY',json.dumps(public))
        self.assertTrue(json.load(a.open(base+'/api/agent/state'))['enabled'])
        self.assertFalse(json.load(b.open(base+'/api/agent/state'))['enabled'])
        stale={'text':'旧配置请求','generation':old['generation'],'runtime_id':old['runtime_id'],'speak':False}
        request=Request(base+'/api/agent/turn',data=json.dumps(stale).encode(),headers={'X-Arm-Desk':'1'})
        with self.assertRaises(HTTPError) as caught:a.open(request)
        self.assertEqual(caught.exception.code,400)
        self.assertEqual(json.load(caught.exception)['error'],'stale_input')

    def test_global_stop_reaches_controller_even_without_browser_sessions(self):
        submitted=[]
        hub=AgentHub();self.addCleanup(hub.close)
        desk=SimpleNamespace(snapshot=lambda:{},submit=lambda *args:submitted.append(args))
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(desk,'FAKE',hub))
        Thread(target=server.serve_forever,daemon=True).start()
        self.addCleanup(server.server_close);self.addCleanup(server.shutdown)
        request=Request(f'http://127.0.0.1:{server.server_port}/api/action',
                        data=b'{"action":"stop"}',headers={'X-Arm-Desk':'1'})
        self.assertEqual(build_opener().open(request).status,202)
        self.assertEqual(submitted,[('stop',None)])


if __name__=='__main__':unittest.main()
