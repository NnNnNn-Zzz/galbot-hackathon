"""Read-only stationary waypoint capture. Run from the handoff directory.

Uses configured SSH key/agent and GALBOT_CAM_CODE or GALBOT_CAM_TOKEN.
No motion, controller switch, deployment or remote file write is performed.
"""
from __future__ import annotations
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time

from medicine_frontend.dataset import Dataset, DEFAULT_ROOT
from medicine_frontend.tools import collect
from medicine_frontend.tools.probe_drawer_force import ssh_prefix

SOURCE = '''import json,time
from galbot_sdk.g1 import GalbotRobot
r=GalbotRobot()
if not r.init(): raise RuntimeError('SDK init failed')
try:
 for i in range(20):
  out={'received_ns':time.time_ns(),'joints':[], 'grippers':{}, 'errors':[]}
  try: out['joints']=[{'name':str(s.joint_name),'position':float(s.position),'velocity':float(s.velocity),'effort':float(s.effort),'timestamp_ns':int(s.timestamp_ns)} for s in r.get_joint_states([],[])]
  except Exception as e: out['errors'].append(type(e).__name__)
  for side in ('left','right'):
   try:
    s=r.get_gripper_state(side+'_gripper')
    out['grippers'][side]={k:s.get(k) if isinstance(s,dict) else getattr(s,k,None) for k in ('width','velocity','effort','is_moving','timestamp_ns')}
   except Exception as e: out['errors'].append(type(e).__name__)
  print('SAMPLE '+json.dumps(out),flush=True)
  time.sleep(.1)
finally:
 r.request_shutdown();r.wait_for_shutdown();r.destroy()
'''

def collect_states(folder):
    command = ('cd /home/galbot/galbot-sdk-web-console && '
               'PYTHONPATH=/data/galbot/lib PYTHONDONTWRITEBYTECODE=1 '
               'timeout 30s .venv/bin/python -u -')
    timeout = False
    try:
        result = subprocess.run([*ssh_prefix(), command], input=SOURCE.encode(),
                                capture_output=True, timeout=60)
        raw, code = result.stdout, result.returncode
    except subprocess.TimeoutExpired as exc:
        raw, code, timeout = exc.stdout or b'', None, True
    samples = []
    for line in raw.decode('utf-8', errors='replace').splitlines():
        if line.startswith('SAMPLE '):
            samples.append(json.loads(line[7:]))
    (folder/'states.ndjson').write_text(''.join(json.dumps(s)+'\n' for s in samples), encoding='utf-8')
    valid = len(samples) == 20 and code == 0 and all(
        not s['errors'] and len(s['joints']) == 25 and
        all(math.isfinite(j['position']) for j in s['joints']) and
        all(s['grippers'].get(side, {}).get('width') is not None for side in ('left','right'))
        for s in samples)
    return {'samples':len(samples), 'exit_code':code, 'timeout':timeout, 'valid':valid}

def collect_cameras(session, root):
    result={}
    for kind in ('wrist','head'):
        result[kind]=collect.main([kind,'--sources',kind+'_right',kind+'_left',
            '--session',session,'--root',str(root),'--arrangement','return_waypoint',
            '--grasp-state','pre_grasp','--note','Operator designated stationary waypoint; actual timestamps retained.',
            '--count','1','--yes'])
    return result

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',required=True)
    parser.add_argument('--point',required=True)
    parser.add_argument('--root',type=Path,default=DEFAULT_ROOT)
    args=parser.parse_args(argv)
    if not re.fullmatch(r'[A-Za-z0-9_-]+',args.session) or not re.fullmatch(r'P[0-9]+',args.point):
        parser.error('Use a safe session name and P0/P1/... point ID')
    if not (os.environ.get('GALBOT_CAM_CODE') or os.environ.get('GALBOT_CAM_TOKEN')):
        parser.error('Set GALBOT_CAM_CODE or GALBOT_CAM_TOKEN in the process environment')
    ssh_prefix()  # preflight configuration before creating a capture
    session=args.session+'_'+args.point
    folder=args.root/'teaching'/session
    folder.mkdir(parents=True,exist_ok=False)
    started=time.time_ns()
    results={}
    with ThreadPoolExecutor(max_workers=2) as pool:
        jobs={'state':pool.submit(collect_states,folder),
              'cameras':pool.submit(collect_cameras,session,args.root)}
        for name,job in jobs.items():
            try: results[name]=job.result()
            except Exception as exc: results[name]={'error':type(exc).__name__}
    with Dataset(args.root) as db:
        frames=[{k:r[k] for k in ('source','image_path','depth_path','quality_flags')}
                for r in db.records() if r['session']==session]
    complete=results.get('state',{}).get('valid') is True and len(frames)==4
    manifest={'point':args.point,'session':session,'started_ns':started,'finished_ns':time.time_ns(),
              'status':'captured_with_quality_flags' if complete else 'partial',
              'results':results,'frames':frames,'motion_commands_sent':[],
              'capture_synchronized':False,'continuous_trajectory_recorded':False}
    (folder/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(manifest,ensure_ascii=False))
    return 0 if complete else 1

if __name__=='__main__':
    raise SystemExit(main())
