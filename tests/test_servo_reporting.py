import tempfile
import unittest
from pathlib import Path
from eai_robot.applications.visual_servo.reporting import summarize,read_records


def motion(stamp=1,actual=7):
    return {'time':stamp,'event':'motion','before':{'base':0},'goal':{'base':10},'actual':{'base':actual}}


class ServoReportingTest(unittest.TestCase):
    def test_counts_only_cycles_backed_by_real_motion_not_initial_alignment(self):
        records=[{'time':0,'event':'locked','completed_cycles':0,'error':[0,0],'tolerance':[64,48]},
                 motion(),{'time':2,'event':'response','error_before':[100,10]},
                 {'time':3,'event':'locked','completed_cycles':1,'error':[10,4],'tolerance':[64,48]}]
        summary,cycles,_=summarize(records)
        self.assertEqual(summary['evidence_backed_cycles'],1)
        self.assertEqual(cycles[0]['first_confirmed_motion_to_aligned_s'],2)
        self.assertEqual(summary['joint_response']['base']['absolute_residual_counts']['median'],3)

    def test_displayed_count_does_not_prove_motion(self):
        records=[motion(actual=0),{'time':2,'event':'locked','completed_cycles':1,'error':[0,0]}]
        summary,cycles,_=summarize(records)
        self.assertEqual(cycles,[])
        self.assertEqual(summary['unsupported_counter_increments'],[1])
        self.assertEqual(summary['invalid_motion_events'],[1])

    def test_nonmonotonic_log_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'events.jsonl'
            path.write_text('{"time":2,"event":"x"}\n{"time":1,"event":"x"}\n')
            with self.assertRaises(ValueError): read_records(path)
