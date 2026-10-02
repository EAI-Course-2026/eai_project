"""Local GUI safety and API tests without opening a serial device."""
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from http.server import ThreadingHTTPServer
from threading import Thread

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from eai_robot.arm.desk_service import ArmDeskService
from eai_robot.arm.desk_server import make_handler
from test_arm import FakeArm


class DeskTests(unittest.TestCase):
    def make_desk(self, bus=None, watchdog=8):
        bus = bus or FakeArm()
        for sid, pos in enumerate((349,81,771,93,812,278), 1):
            bus.registers[sid]['Present_Position'] = pos
        service = ArmDeskService(ROOT/'calibration/scs215_so101.json',
                                 ROOT/'configs/demo_home.json',
                                 backend_factory=lambda kind,port,baud:bus,
                                 idle_poll_seconds=.02,watchdog_seconds=watchdog)
        self.addCleanup(service.close)
        return service,bus

    def settle(self, service, timeout=5, heartbeat=True):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            state=service.snapshot() if heartbeat else service.state.copy()
            if not state['busy']:
                return state
            time.sleep(.01)
        self.fail('hardware worker did not settle')

    def connect(self, service):
        service.submit('connect',{'backend':'serial','port':'FAKE','baudrate':1_000_000})
        state=self.settle(service)
        self.assertIsNone(state['error'],state['error'])
        self.assertTrue(state['connected'])
        return state

    def test_connect_is_read_only_and_enable_gates_manual_control(self):
        service,bus=self.make_desk()
        bus.registers[5]['Present_Position']=235
        state=self.connect(service)
        self.assertEqual(len(state['joints']),6)
        self.assertFalse(state['control_enabled'])
        self.assertEqual(bus.writes,[])
        self.assertEqual(bus.packets,[])
        with self.assertRaisesRegex(RuntimeError,'启用控制'):
            service.submit('move',{'values':[.5]*6})
        original={sid:bus.read(sid,'Present_Position') for sid in range(1,7)}
        service.submit('enable_control')
        state=self.settle(service)
        self.assertIsNone(state['error'])
        self.assertTrue(state['control_enabled'])
        self.assertFalse(state['homed'])
        self.assertEqual(bus.packets[0],original)
        self.assertEqual(state['targets']['5'],235)
        # A rejected optional home leaves ordinary control enabled.
        with self.assertRaisesRegex(ValueError,'归位入口'):
            service.submit('home')
        self.assertTrue(service.snapshot()['control_enabled'])
        with self.assertRaisesRegex(ValueError,'普通控制范围'):
            service.submit('move',{'values':[1,.5,.5,.5,.5,.5]})
        # Full travel is no longer intersected with the saved home radius.
        values=[.90,.85,.15,.85,.20,.80]
        service.submit('move',{'values':values})
        state=self.settle(service)
        self.assertIsNone(state['error'],state['error'])
        self.assertEqual(state['motion_state'],'reached')
        expected={cal.id:cal.position(v) for cal,v in zip(service.calibration.values(),values)}
        self.assertEqual(bus.packets[-1],expected)
        for previous,following in zip(bus.packets,bus.packets[1:]):
            self.assertLessEqual(max(abs(following[i]-previous[i]) for i in following),20)
        service.submit('stop')
        state=self.settle(service)
        self.assertFalse(state['control_enabled'])
        self.assertFalse(state['torque_on'])
        with self.assertRaisesRegex(RuntimeError,'重新启用'):
            service.submit('move',{'values':[.5]*6})
        service.submit('enable_control')
        self.assertTrue(self.settle(service)['control_enabled'])

    def test_jog_uses_feedback_and_rejects_boundary_without_release(self):
        service,bus=self.make_desk()
        self.connect(service)
        service.submit('enable_control');self.settle(service)
        before={sid:bus.read(sid,'Present_Position') for sid in range(1,7)}
        service.submit('jog',{'joint':'wrist_roll','delta':-5})
        state=self.settle(service)
        expected=dict(before);expected[5]-=5
        self.assertEqual(bus.packets[-1],expected)
        self.assertIsNone(state['error'])
        bus.registers[5]['Present_Position']=service.calibration['wrist_roll'].range_min+20
        service._refresh()
        count=len(bus.packets)
        with self.assertRaisesRegex(ValueError,'点动目标'):
            service.submit('jog',{'joint':'wrist_roll','delta':-5})
        self.assertEqual(len(bus.packets),count)
        self.assertTrue(service.snapshot()['control_enabled'])
        with self.assertRaises(ValueError):
            service.submit('jog',{'joint':'wrist_roll','delta':50})

    def test_screenshot_pose_can_take_over_and_move_only_edited_joint(self):
        service,bus=self.make_desk()
        pose=(531,222,514,170,442,249)
        for sid,raw in enumerate(pose,1):
            bus.registers[sid]['Present_Position']=raw
        self.connect(service)
        service.submit('enable_control')
        state=self.settle(service)
        self.assertTrue(state['control_enabled'],state['error'])
        self.assertEqual(bus.packets[0],dict(enumerate(pose,1)))
        values=[service.calibration[name].ratio(raw) for name,raw in zip(service.calibration,pose)]
        values[0]=.60
        service.submit('move',{'values':values,'joints':['shoulder_pan']})
        state=self.settle(service)
        self.assertIsNone(state['error'])
        self.assertTrue(all(packet[6]==249 for packet in bus.packets))
        self.assertEqual(tuple(bus.read(s,'Present_Position') for s in range(2,7)),pose[1:])
        # In the endpoint margin, only inward small steps are allowed.
        count=len(bus.packets)
        with self.assertRaisesRegex(ValueError,'内部点动'):
            service.submit('jog',{'joint':'gripper','delta':-5})
        self.assertEqual(len(bus.packets),count)
        self.assertTrue(service.snapshot()['control_enabled'])
        service.submit('jog',{'joint':'gripper','delta':5})
        self.assertIsNone(self.settle(service)['error'])
        self.assertEqual(bus.read(6,'Present_Position'),254)
        service.submit('jog',{'joint':'gripper','delta':5})
        self.assertIsNone(self.settle(service)['error'])
        self.assertEqual(bus.read(6,'Present_Position'),259)
        for joints in ([],['gripper','gripper'],['unknown']):
            with self.assertRaises(ValueError):
                service.submit('move',{'values':values,'joints':joints})

    def test_takeover_rejects_outside_calibration_without_goal_write(self):
        service,bus=self.make_desk()
        bus.registers[5]['Present_Position']=46
        self.connect(service)
        service.submit('enable_control')
        state=self.settle(service)
        self.assertIn('接管范围',state['error'])
        self.assertFalse(state['control_enabled'])
        self.assertEqual(bus.packets,[])
        self.assertEqual(bus.writes,[])

    def test_takeover_checks_fault_and_cleans_up_partial_enable(self):
        service,bus=self.make_desk()
        self.connect(service)
        bus.fail_enable=3
        service.submit('enable_control')
        state=self.settle(service)
        self.assertFalse(state['control_enabled'])
        self.assertEqual({bus.read(i,'Torque_Enable') for i in range(1,7)},{0})
        self.assertEqual(state['motion_state'],'failed')
        bus.fail_enable=None
        bus.registers[5]['Status']=1
        count=len(bus.packets)
        service.submit('enable_control')
        self.assertIn('报警',self.settle(service)['error'])
        self.assertEqual(len(bus.packets),count)

    def test_stop_during_takeover_prevents_remaining_enable_writes(self):
        from threading import Event
        entered,proceed=Event(),Event()
        class PausedArm(FakeArm):
            def sync_positions(self,targets):
                super().sync_positions(targets)
                entered.set()
                proceed.wait(2)
        service,bus=self.make_desk(PausedArm())
        self.connect(service)
        service.submit('enable_control')
        self.assertTrue(entered.wait(2))
        service.submit('stop')
        proceed.set()
        state=self.settle(service)
        self.assertFalse(state['control_enabled'])
        self.assertFalse(state['torque_on'])
        self.assertFalse(any(r=='Torque_Enable' and v==1 for _,r,v in bus.writes))

    def test_missing_home_and_reverse_mapping_do_not_limit_manual_control(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'reversed.json'
            data=json.loads((ROOT/'calibration/scs215_so101.json').read_text())
            data['wrist_roll']['drive_mode']=1
            path.write_text(json.dumps(data))
            bus=FakeArm()
            service=ArmDeskService(path,Path(d)/'absent-home.json',backend_factory=lambda *_:bus,idle_poll_seconds=.02)
            self.addCleanup(service.close)
            self.connect(service)
            service.submit('enable_control')
            self.assertTrue(self.settle(service)['control_enabled'])
            service.submit('move',{'values':[.5,.5,.5,.5,.8,.5]})
            state=self.settle(service)
            self.assertIsNone(state['error'])
            self.assertEqual(bus.packets[-1][5],service.calibration['wrist_roll'].position(.8))
            self.assertAlmostEqual(state['joints'][4]['ratio'],.8,delta=.001)
            self.assertEqual((state['calibration'][4]['safe_min'],state['calibration'][4]['safe_max']),(954,67))

    def test_hold_is_written_and_verified_before_any_torque_enable(self):
        class OrderedArm(FakeArm):
            def write(self,sid,register,value):
                if register=='Torque_Enable' and value==1:
                    assert self.packets
                    assert all(self.read(i,'Goal_Position')==self.packets[0][i] for i in range(1,7))
                super().write(sid,register,value)
        service,bus=self.make_desk(OrderedArm())
        self.connect(service)
        service.submit('enable_control')
        self.assertTrue(self.settle(service)['control_enabled'])

    def test_motion_timeout_releases_and_requires_new_takeover(self):
        service,bus=self.make_desk()
        self.connect(service)
        service.submit('enable_control');self.settle(service)
        bus.follow=False
        service.arm.timeout=.15
        service.submit('move',{'values':[.5]*6})
        state=self.settle(service)
        self.assertIsNotNone(state['error'])
        self.assertFalse(state['control_enabled'])
        self.assertFalse(state['torque_on'])
        self.assertEqual(state['motion_state'],'failed')

    def test_jog_requires_actual_small_step_arrival(self):
        service,bus=self.make_desk()
        self.connect(service)
        service.submit('enable_control');self.settle(service)
        bus.follow=False
        service.arm.timeout=.15
        service.submit('jog',{'joint':'wrist_roll','delta':5})
        state=self.settle(service)
        self.assertIn('点动未到位',state['error'])
        self.assertFalse(state['control_enabled'])
        self.assertFalse(state['torque_on'])

    def test_gravity_drift_homes_without_commanding_below_hardware_limit(self):
        service,bus=self.make_desk()
        lower=service.calibration['shoulder_lift'].range_min
        bus.registers[2]['Present_Position']=lower-1
        for c in service.calibration.values():
            bus.registers[c.id]['Min_Position_Limit']=c.range_min
            bus.registers[c.id]['Max_Position_Limit']=c.range_max
        self.connect(service)
        service.submit('home')
        state=self.settle(service)
        self.assertTrue(state['homed'],state['error'])
        self.assertEqual(bus.packets[0][2],lower)
        self.assertTrue(all(packet[2]>=lower for packet in bus.packets))
        service.submit('stop')
        self.assertFalse(self.settle(service)['torque_on'])

    def test_stop_interrupts_active_motion_and_releases_all(self):
        class SlowArm(FakeArm):
            def sync_positions(self, targets):
                time.sleep(.05)
                super().sync_positions(targets)
        service,bus=self.make_desk(SlowArm())
        self.connect(service)
        service.submit('home')
        self.assertTrue(self.settle(service)['homed'])
        service.submit('move',{'values':[.25,.16,.80,.15,.78,.78]})
        time.sleep(.08)
        self.assertTrue(service.snapshot()['busy'])
        service.submit('stop')
        self.settle(service)
        state=self.settle(service)
        self.assertFalse(state['torque_on'])
        self.assertFalse(state['homed'])
        self.assertEqual({bus.read(s,'Torque_Enable') for s in range(1,7)},{0})

    def test_watchdog_releases_if_browser_disappears(self):
        service,bus=self.make_desk(watchdog=.35)
        self.connect(service)
        service.submit('home')
        self.assertTrue(self.settle(service)['homed'])
        # Stop sending browser heartbeats.
        deadline=time.monotonic()+2
        while time.monotonic()<deadline and service.state['torque_on']:
            time.sleep(.02)
        self.assertFalse(service.state['torque_on'])
        self.assertEqual({bus.read(s,'Torque_Enable') for s in range(1,7)},{0})

    def test_capture_and_save_calibration_without_eeprom_write(self):
        service,bus=self.make_desk()
        self.connect(service)
        with tempfile.TemporaryDirectory() as d, patch('eai_robot.arm.desk_service.ROOT',Path(d)):
            for sid,name in enumerate(('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'),1):
                for endpoint,raw in enumerate((300,500)):
                    bus.registers[sid]['Present_Position']=raw
                    service.submit('capture',{'joint':name,'endpoint':endpoint})
                    self.assertIsNone(self.settle(service)['error'])
            service.submit('save_calibration')
            state=self.settle(service)
            self.assertIsNone(state['error'],state['error'])
            self.assertEqual(len([p for p in (Path(d)/'calibration').glob('scs215_manual_*.json') if not p.name.endswith('.meta.json')]),1)
            self.assertEqual(bus.writes,[])

    def test_manual_control_works_without_matching_home(self):
        with tempfile.TemporaryDirectory() as d:
            base=Path(d)
            data=json.loads((ROOT/'calibration/scs215_so101.json').read_text())
            data['shoulder_pan']['range_min'] += 1
            path=base/'new_calibration.json'; path.write_text(json.dumps(data))
            bus=FakeArm()
            for sid,pos in enumerate((360,174,664,162,789,420),1):
                bus.registers[sid]['Present_Position']=pos
            service=ArmDeskService(path,ROOT/'configs/demo_home.json',
                                   backend_factory=lambda *_:bus,idle_poll_seconds=.02)
            self.addCleanup(service.close)
            self.assertTrue(service.snapshot()['home_required'])
            self.connect(service)
            with self.assertRaisesRegex(RuntimeError,'匹配起点'):
                service.submit('home')
            self.assertEqual(bus.writes,[])
            service.submit('enable_control')
            self.assertTrue(self.settle(service)['control_enabled'])
            service.submit('move',{'values':[.5]*6})
            self.assertIsNone(self.settle(service)['error'])
            service.submit('stop');self.settle(service)
            # Reset the fake pose to a teachable demonstration pose.
            for sid,pos in enumerate((360,174,664,162,789,420),1):
                bus.registers[sid]['Present_Position']=pos
            with patch('eai_robot.arm.desk_service.ROOT',base):
                service.submit('teach_home')
                state=self.settle(service)
            self.assertIsNone(state['error'],state['error'])
            self.assertFalse(state['home_required'])
            self.assertTrue(list((base/'configs').glob('demo_home_*.local.json')))
            service.submit('home')
            self.assertTrue(self.settle(service)['homed'])
            service.submit('stop')
            self.assertFalse(self.settle(service)['torque_on'])

    def test_local_http_requires_header_and_serves_six_axis_ui(self):
        service,_=self.make_desk()
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(service,'FAKE'))
        thread=Thread(target=server.serve_forever,daemon=True);thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        base=f'http://127.0.0.1:{server.server_port}'
        with urlopen(base+'/') as response:
            html=response.read().decode()
        self.assertIn('六关节控制',html)
        self.assertIn('data-demo="transfer"',html)
        with urlopen(base+'/api/state') as response:
            self.assertEqual(len(json.load(response)['calibration']),6)
        with patch('eai_robot.arm.desk_server.discover_ports',return_value=[{'device':'FAKE','description':'USB serial','manufacturer':'test'}]):
            with urlopen(base+'/api/ports') as response:
                self.assertEqual(json.load(response)['ports'][0]['device'],'FAKE')
        with patch('eai_robot.arm.desk_server.discover_ports',return_value=[]):
            with urlopen(base+'/api/ports') as response:
                self.assertEqual(json.load(response)['ports'],[])
        data=json.dumps({'action':'connect','payload':{'backend':'serial','port':'FAKE','baudrate':1_000_000}}).encode()
        with self.assertRaises(HTTPError) as error:
            urlopen(Request(base+'/api/action',data=data,headers={'Content-Type':'application/json'}))
        self.assertEqual(error.exception.code,403)
        request=Request(base+'/api/action',data=data,headers={'Content-Type':'application/json','X-Arm-Desk':'1'})
        with urlopen(request) as response:
            self.assertEqual(response.status,202)
        self.assertTrue(self.settle(service)['connected'])


if __name__=='__main__':unittest.main()
