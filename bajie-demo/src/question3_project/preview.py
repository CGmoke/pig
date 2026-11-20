"""Optional, renderer-free live telemetry for the local Q3 viewer."""
import json
from pathlib import Path
from time import monotonic


class Feed:
    def __init__(self, path, now=monotonic):
        self.path = Path(path)
        self.pending = self.path.with_suffix('.tmp')
        self.now = now
        self.last = float('-inf')
        self.sequence = 0
        self.hits = {}

    def publish(self, case_id, toy, sample, *, force=False, contact=None):
        now = self.now()
        if not (force or contact or now - self.last >= 1 / 16):
            return
        if contact:
            self.hits[case_id] = contact
        frame = dict(sample(), id=case_id, toy=toy, contact=contact,
                     hits=self.hits, sequence=self.sequence + 1)
        self.pending.write_text(json.dumps(frame, allow_nan=False), encoding='utf-8')
        self.pending.replace(self.path)
        self.sequence += 1
        self.last = now
