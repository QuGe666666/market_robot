"""Protocol smoke test for the resident backend (no ROS or robot required)."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORKER = ROOT / "scripts" / "grasp_resident_worker.py"


def test_two_requests_keep_independent_arguments(tmp_path: Path) -> None:
    fake_script = tmp_path / "fake_grasp.py"
    fake_script.write_text(
        """
import argparse
import json
from pathlib import Path

COUNTER = Path(__file__).with_suffix('.count')
count = int(COUNTER.read_text()) if COUNTER.exists() else 0
COUNTER.write_text(str(count + 1))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--arm', required=True)
    parser.add_argument('--box', required=True)
    parser.add_argument('--angle', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    Path(args.output).write_text(json.dumps(vars(args), sort_keys=True))
    print('REQUEST arm=' + args.arm + ' box=' + args.box + ' angle=' + args.angle, flush=True)
    return 0
"""
    )
    out_left = tmp_path / "left.json"
    out_right = tmp_path / "right.json"
    requests = [
        {"request_id": "left-1", "argv": ["--arm", "left", "--box", "y", "--angle", "30", "--output", str(out_left)]},
        {"request_id": "right-1", "argv": ["--arm", "right", "--box", "n", "--angle", "60", "--output", str(out_right)]},
    ]
    proc = subprocess.Popen(
        [sys.executable, str(WORKER), str(fake_script)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    assert proc.stdin is not None and proc.stdout is not None
    assert proc.stdout.readline().strip() == "__GRASP_RESIDENT_READY__"
    for request in requests:
        proc.stdin.write(json.dumps(request) + "\n")
        proc.stdin.flush()
        result = None
        for line in proc.stdout:
            if line.startswith("__GRASP_RESIDENT_RESULT__"):
                result = json.loads(line[len("__GRASP_RESIDENT_RESULT__"):])
                break
        assert result == {"request_id": request["request_id"], "return_code": 0}
    proc.stdin.close()
    proc.terminate()
    proc.wait(timeout=5)
    assert json.loads(out_left.read_text())["box"] == "y"
    assert json.loads(out_left.read_text())["angle"] == "30"
    assert json.loads(out_right.read_text())["box"] == "n"
    assert json.loads(out_right.read_text())["angle"] == "60"
    # The module was imported once; both requests ran in that same interpreter.
    assert fake_script.with_suffix(".count").read_text() == "1"
