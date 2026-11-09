"""Offline file-integrity check; never contacts the robot."""
import hashlib
import json
from pathlib import Path

root=Path(__file__).resolve().parent
manifest=json.loads((root/'SHA256SUMS.json').read_text(encoding='utf-8'))
for entry in manifest['files']:
    p=(root/entry['path']).resolve()
    p.relative_to(root)
    raw=p.read_bytes()
    if len(raw)!=entry['bytes'] or hashlib.sha256(raw).hexdigest()!=entry['sha256']:
        raise SystemExit('Integrity failure: '+entry['path'])
print('Verified',len(manifest['files']),'files')
