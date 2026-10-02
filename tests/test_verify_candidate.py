"""Candidate verification script: fake bus, no physical serial."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from eai_robot.arm.calibration import JOINTS,load
from test_arm import FakeArm

spec=importlib.util.spec_from_file_location('arm_verify_candidate',ROOT/'scripts/arm_verify_candidate.py')
verify=importlib.util.module_from_spec(spec)
spec.loader.exec_module(verify)


class CandidateVerifyTests(unittest.TestCase):
    def run_candidate(self, argv):
        return verify.run([
            '--calibration', str(ROOT/'calibration/history/scs215_safe_candidate_20261002.json'),
            '--home-file', str(ROOT/'calibration/history/demo_home_scs215_safe_candidate_20261002.json'),
            *argv,
        ])

    def bus(self):
        bus=FakeArm()
        old=load(ROOT/'calibration/scs215_so101.json')
        for name in JOINTS:
            c=old[name]
            bus.registers[c.id]['Min_Position_Limit']=c.range_min
            bus.registers[c.id]['Max_Position_Limit']=c.range_max
        for sid,value in enumerate((360,63,664,162,789,420),1):
            bus.registers[sid]['Present_Position']=value
        return bus,old

    def test_recovery_homing_and_endpoint_roundtrip_are_logged_and_release(self):
        bus,old=self.bus()
        with tempfile.TemporaryDirectory() as d, patch.object(verify,'SerialBackend',return_value=bus), patch.object(verify.time,'sleep'):
            log=Path(d)/'verify.jsonl'
            code=self.run_candidate(['--port','FAKE','--joint','shoulder_lift','--enable','--log',str(log)])
            events=[json.loads(line) for line in log.read_text().splitlines()]
        self.assertEqual(code,0)
        self.assertTrue(any(e['event']=='startup_recover' for e in events))
        self.assertEqual(len([e for e in events if e['event']=='endpoint']),2)
        self.assertTrue(any(e['event']=='complete' for e in events))
        self.assertEqual(set(events[-1]['torque'].values()),{0})
        for packet in bus.packets:
            for name in JOINTS:
                c=old[name]
                self.assertTrue(c.range_min<=packet[c.id]<=c.range_max)
        self.assertTrue(bus.closed)

    def test_read_only_mode_has_no_writes(self):
        bus,_=self.bus()
        with patch.object(verify,'SerialBackend',return_value=bus):
            code=self.run_candidate(['--port','FAKE','--joint','shoulder_lift'])
        self.assertEqual(code,0)
        self.assertEqual(bus.packets,[])
        self.assertEqual(bus.writes,[])
        self.assertTrue(bus.closed)

    def test_default_preflight_selects_active_calibration_and_has_no_writes(self):
        bus,_=self.bus()
        with patch.object(verify,'SerialBackend',return_value=bus), \
                patch.object(verify,'load',wraps=load) as loader:
            code=verify.run(['--port','FAKE'])
        self.assertEqual(code,0)
        self.assertTrue(all(call.args[0] == verify.CURRENT for call in loader.call_args_list))
        self.assertEqual(bus.packets,[])
        self.assertEqual(bus.writes,[])
        self.assertTrue(bus.closed)

    def test_return_home_from_outside_demo_entrance(self):
        bus,_=self.bus()
        bus.registers[2]['Present_Position']=554
        with tempfile.TemporaryDirectory() as d, patch.object(verify,'SerialBackend',return_value=bus), patch.object(verify.time,'sleep'):
            log=Path(d)/'return.jsonl'
            code=self.run_candidate(['--port','FAKE','--return-home','--enable','--log',str(log)])
            events=[json.loads(line) for line in log.read_text().splitlines()]
        self.assertEqual(code,0)
        self.assertFalse(any(e['event']=='endpoint' for e in events))
        self.assertEqual(bus.registers[2]['Present_Position'],174)
        self.assertEqual(set(events[-1]['torque'].values()),{0})

    def test_clearance_pose_precedes_elbow_homing(self):
        bus,_=self.bus()
        bus.registers[3]['Present_Position']=771
        with tempfile.TemporaryDirectory() as d, patch.object(verify,'SerialBackend',return_value=bus), patch.object(verify.time,'sleep'):
            log=Path(d)/'clearance.jsonl'
            code=self.run_candidate(['--port','FAKE','--return-home','--clearance-wrist','240',
                             '--clearance-gripper','520','--enable','--log',str(log)])
            events=[json.loads(line) for line in log.read_text().splitlines()]
        self.assertEqual(code,0)
        stages=[e for e in events if e['event']=='stage_start']
        self.assertEqual([e['stage'] for e in stages],['home_proximal','home_distal'])
        clearance=[e for e in events if e['event']=='startup_clearance']
        self.assertEqual([e['id'] for e in clearance],sorted(e['id'] for e in clearance))
        self.assertEqual(stages[0]['target']["4"],240)
        self.assertEqual(stages[0]['target']["6"],520)
        self.assertEqual(set(events[-1]['torque'].values()),{0})

    def test_clearance_is_held_through_endpoint_run_then_restored(self):
        bus,_=self.bus()
        with tempfile.TemporaryDirectory() as d, patch.object(verify,'SerialBackend',return_value=bus), patch.object(verify.time,'sleep'):
            log=Path(d)/'clearance_verify.jsonl'
            code=self.run_candidate(['--port','FAKE','--joint','shoulder_lift',
                             '--clearance-wrist','240','--clearance-gripper','520',
                             '--enable','--log',str(log)])
            events=[json.loads(line) for line in log.read_text().splitlines()]
        self.assertEqual(code,0)
        stages=[e for e in events if e['event']=='stage_start']
        self.assertEqual(stages[1]['target']["4"],240)
        self.assertEqual(stages[1]['target']["6"],520)
        self.assertEqual(stages[-1]['stage'],'restore_original_home')
        self.assertEqual(stages[-1]['target']["4"],162)
        self.assertEqual(stages[-1]['target']["6"],420)

    def test_manual_start_accepts_placed_pose_outside_demo_entry(self):
        bus,_=self.bus()
        bus.registers[2]['Present_Position']=174
        bus.registers[3]['Present_Position']=545
        bus.registers[4]['Present_Position']=329
        bus.registers[6]['Present_Position']=518
        with tempfile.TemporaryDirectory() as d, patch.object(verify,'SerialBackend',return_value=bus), patch.object(verify.time,'sleep'):
            log=Path(d)/'manual_verify.jsonl'
            code=self.run_candidate(['--port','FAKE','--manual-start','--joint','shoulder_lift',
                             '--clearance-wrist','330','--clearance-gripper','520',
                             '--enable','--log',str(log)])
            events=[json.loads(line) for line in log.read_text().splitlines()]
        self.assertEqual(code,0)
        self.assertTrue(any(e['event']=='endpoint' for e in events))
        self.assertEqual(set(events[-1]['torque'].values()),{0})


if __name__=='__main__':unittest.main()
