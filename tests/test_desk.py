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

    def test_connect_is_read_only_and_home_gates_manual_control(self):
        service,bus=self.make_desk()
        state=self.connect(service)
        self.assertEqual(len(state['joints']),6)
        self.assertFalse(state['homed'])
        self.assertEqual(bus.writes,[])
        self.assertEqual(bus.packets,[])
        with self.assertRaisesRegex(RuntimeError,'先进入统一起点'):
            service.submit('move',{'values':[service.calibration[n].ratio(service.home[i]) for i,n in enumerate(('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'),1)]})
        service.submit('home')
        state=self.settle(service)
        self.assertTrue(state['homed'],state['error'])
        self.assertEqual(state['torque_on'],True)
        with self.assertRaisesRegex(ValueError,'入口'):
            service.submit('move',{'values':[1,.5,.5,.5,.5,.5]})
        self.assertTrue(service.snapshot()['homed'])
        service.submit('stop')
        state=self.settle(service)
        self.assertFalse(state['torque_on'])
        self.assertEqual({bus.read(s,'Torque_Enable') for s in range(1,7)},{0})

    def test_gravity_drift_to_63_homes_without_commanding_below_hardware_limit(self):
        service,bus=self.make_desk()
        bus.registers[2]['Present_Position']=63
        self.connect(service)
        service.submit('home')
        state=self.settle(service)
        self.assertTrue(state['homed'],state['error'])
        self.assertEqual(bus.packets[0][2],64)
        self.assertTrue(all(packet[2]>=64 for packet in bus.packets))
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

    def test_new_calibration_stays_read_only_until_matching_home_is_taught(self):
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
        data=json.dumps({'action':'connect','payload':{'backend':'serial','port':'FAKE','baudrate':1_000_000}}).encode()
        with self.assertRaises(HTTPError) as error:
            urlopen(Request(base+'/api/action',data=data,headers={'Content-Type':'application/json'}))
        self.assertEqual(error.exception.code,403)
        request=Request(base+'/api/action',data=data,headers={'Content-Type':'application/json','X-Arm-Desk':'1'})
        with urlopen(request) as response:
            self.assertEqual(response.status,202)
        self.assertTrue(self.settle(service)['connected'])


if __name__=='__main__':unittest.main()
