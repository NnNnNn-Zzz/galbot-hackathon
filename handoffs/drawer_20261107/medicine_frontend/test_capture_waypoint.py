import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
from medicine_frontend.tools import capture_waypoint as capture

class CaptureWaypointTests(unittest.TestCase):
    def sample_bytes(self, count=20):
        sample={'received_ns':1,'joints':[{'name':str(i),'position':0.0} for i in range(25)],
                'grippers':{'left':{'width':0.05},'right':{'width':0.05}},'errors':[]}
        return ''.join('SAMPLE '+json.dumps(sample)+'\n' for _ in range(count)).encode()

    @patch.dict('os.environ', {'GALBOT_SSH_TARGET':'test-host'})
    def test_complete_numeric_samples_are_saved(self):
        with tempfile.TemporaryDirectory() as t, patch.object(capture.subprocess,'run') as run:
            run.return_value=subprocess.CompletedProcess([],0,stdout=self.sample_bytes())
            result=capture.collect_states(Path(t))
            self.assertTrue(result['valid'])
            self.assertEqual(len((Path(t)/'states.ndjson').read_text().splitlines()),20)
            self.assertIn('BatchMode=yes',run.call_args.args[0])

    @patch.dict('os.environ', {'GALBOT_SSH_TARGET':'test-host'})
    def test_timeout_preserves_partial_samples_and_is_invalid(self):
        with tempfile.TemporaryDirectory() as t, patch.object(capture.subprocess,'run') as run:
            run.side_effect=subprocess.TimeoutExpired(['ssh'],60,output=self.sample_bytes(2))
            result=capture.collect_states(Path(t))
            self.assertFalse(result['valid'])
            self.assertTrue(result['timeout'])
            self.assertEqual(result['samples'],2)

    @patch.dict('os.environ', {'GALBOT_SSH_TARGET':'test-host','GALBOT_CAM_CODE':'test-only'})
    def test_existing_point_refuses_before_any_remote_call(self):
        with tempfile.TemporaryDirectory() as t, patch.object(capture.subprocess,'run') as run:
            (Path(t)/'teaching/demo_P3').mkdir(parents=True)
            with self.assertRaises(FileExistsError):
                capture.main(['--session','demo','--point','P3','--root',t])
            run.assert_not_called()

if __name__=='__main__':
    unittest.main()
