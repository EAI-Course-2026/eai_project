"""Desktop Agent state, actual HTTP boundary and cancel-before-start races."""
from http.server import ThreadingHTTPServer
import json
from threading import Event, Thread
import time
from types import SimpleNamespace
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from eai_robot.applications.agent.demo import DemoProvider
from eai_robot.applications.desktop.agent import DeskAgent
from eai_robot.applications.desktop.server import make_handler
from eai_robot.policy.agents.loop import AgentSession
from eai_robot.policy.agents.tools import ToolRegistry
from test_cloud_speech import wav_bytes


class Session:
    def __init__(self):
        self.calls, self.cancelled = [], 0
    def run_turn(self, text, **options):
        self.calls.append(text)
        return SimpleNamespace(status='completed', text='演练回答', error_code=None)
    def cancel(self):
        self.cancelled += 1
    def close(self):
        pass


class Speech:
    def transcribe(self, audio, *, cancel):
        return {'text':'查看状态'}
    def synthesize(self, text, *, cancel):
        return {'audio':wav_bytes()}
    def close(self):
        pass


class DeskAgentTests(unittest.TestCase):
    def settle(self, agent):
        agent._thread.join(3)
        self.assertFalse(agent._thread.is_alive())
        return agent.snapshot()

    def test_transcribed_final_text_uses_same_session_and_audio_is_revoked_on_stop(self):
        session, stopped = Session(), []
        agent = DeskAgent(session, Speech(), lambda: stopped.append(True))
        self.addCleanup(agent.close)
        agent.submit(audio=wav_bytes())
        state = self.settle(agent)
        self.assertEqual(session.calls, ['查看状态'])
        self.assertEqual([m['role'] for m in state['messages']], ['user','assistant'])
        self.assertTrue(agent.audio(state['generation']))
        agent.stop()
        self.assertIsNone(agent.audio(state['generation']))
        self.assertTrue(stopped)

    def test_busy_input_not_queued_and_late_asr_result_cannot_reenter_harness(self):
        entered, release = Event(), Event()
        class SlowSpeech(Speech):
            def transcribe(self, audio, *, cancel):
                entered.set(); release.wait(3)
                return {'text':'旧命令'}
        session = Session(); agent = DeskAgent(session, SlowSpeech())
        self.addCleanup(agent.close)
        agent.submit(audio=wav_bytes()); self.assertTrue(entered.wait(2))
        with self.assertRaisesRegex(ValueError, 'busy'):
            agent.submit(text='不要排队')
        agent.stop(); release.set(); self.settle(agent)
        self.assertEqual(session.calls, [])
        self.assertEqual(agent.snapshot()['messages'], [])

    def test_external_cancel_before_core_start_never_calls_model(self):
        class ForbiddenProvider(DemoProvider):
            def stream(self, request, *, cancel):
                self.fail = True
                raise AssertionError('Cancelled input reached API')
        provider = ForbiddenProvider(); provider.fail = False
        session = AgentSession(provider, ToolRegistry([]))
        self.addCleanup(session.close)
        cancel = Event(); cancel.set()
        self.assertEqual(session.run_turn('旧输入', cancellation=cancel).status,'cancelled')
        self.assertFalse(provider.fail)

    def test_tts_error_does_not_repeat_model_or_discard_final_text(self):
        class BrokenSpeech(Speech):
            def synthesize(self, *args, **kwargs):
                raise RuntimeError('PRIVATE_REMOTE_ERROR')
        session = Session(); agent = DeskAgent(session, BrokenSpeech())
        self.addCleanup(agent.close)
        agent.submit(text='查看状态'); state = self.settle(agent)
        self.assertEqual(session.calls,['查看状态'])
        self.assertEqual(state['messages'][-1]['text'],'演练回答')
        self.assertEqual(state['error_code'],'agent_service_error')
        self.assertNotIn('PRIVATE_REMOTE_ERROR',json.dumps(state))

    def test_request_delayed_until_after_stop_cannot_start_new_turn(self):
        session=Session();agent=DeskAgent(session)
        self.addCleanup(agent.close)
        generation=agent.snapshot()['generation']
        agent.stop()
        with self.assertRaisesRegex(ValueError,'stale_input'):
            agent.submit(text='迟到的控制请求',expected_generation=generation)
        self.assertEqual(session.calls,[])

    def test_new_http_routes_keep_origin_header_host_guards_and_never_return_credentials(self):
        session = Session(); agent = DeskAgent(session)
        self.addCleanup(agent.close)
        desk = SimpleNamespace(snapshot=lambda: {}, submit=lambda *args: None)
        server = ThreadingHTTPServer(('127.0.0.1',0),make_handler(desk,'FAKE',agent))
        thread = Thread(target=server.serve_forever,daemon=True);thread.start()
        self.addCleanup(server.server_close);self.addCleanup(server.shutdown)
        base = f'http://127.0.0.1:{server.server_port}'
        body=json.dumps({'text':'查看状态','speak':False,'generation':agent.snapshot()['generation']}).encode()
        for headers in ({}, {'X-Arm-Desk':'1','Origin':'https://evil.invalid'}, {'X-Arm-Desk':'1','Host':'evil.invalid'}):
            with self.assertRaises(HTTPError) as caught:
                urlopen(Request(base+'/api/agent/turn',data=body,headers=headers))
            self.assertEqual(caught.exception.code,403)
        request=Request(base+'/api/agent/turn',data=body,headers={'Content-Type':'application/json','X-Arm-Desk':'1'})
        self.assertEqual(urlopen(request).status,202)
        self.settle(agent)
        state=json.load(urlopen(base+'/api/agent/state'))
        self.assertTrue(state['enabled']); self.assertEqual(state['mode'],'dry_run')
        self.assertNotIn('api_key',json.dumps(state))
        for audio in (None, 123, {}):
            bad = json.dumps({'audio':audio, 'generation':state['generation']}).encode()
            with self.assertRaises(HTTPError) as caught:
                urlopen(Request(base+'/api/agent/turn',data=bad,headers={'X-Arm-Desk':'1'}))
            self.assertEqual(caught.exception.code,400)
            self.assertEqual(json.load(caught.exception)['error'],'invalid_input')
        with self.assertRaises(HTTPError):
            urlopen(base+'/api/agent/audio/9999')


if __name__ == '__main__':
    unittest.main()
