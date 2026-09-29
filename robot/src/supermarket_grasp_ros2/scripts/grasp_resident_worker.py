#!/usr/bin/env python3
"""Long-lived backend for grasp.py with request-isolated command lines."""

from __future__ import annotations

import importlib.util
import json
import os
import sys
import traceback


RESULT_PREFIX = "__GRASP_RESIDENT_RESULT__"


def _load_script(path: str):
    script_path = os.path.abspath(os.path.expanduser(path))
    script_dir = os.path.dirname(script_path)
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)
    spec = importlib.util.spec_from_file_location("resident_grasp_script", script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load grasp script: {script_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: grasp_resident_worker.py SCRIPT_PATH", file=sys.stderr)
        return 2
    module = _load_script(sys.argv[1])
    print("__GRASP_RESIDENT_READY__", flush=True)
    for line in sys.stdin:
        request = None
        try:
            request = json.loads(line)
            argv = request.get("argv")
            request_id = str(request.get("request_id", ""))
            if not isinstance(argv, list) or not all(isinstance(item, str) for item in argv):
                raise ValueError("resident request argv must be a string list")
            old_argv = sys.argv
            sys.argv = [module.__file__ or "grasp.py", *argv]
            try:
                try:
                    result = module.main()
                    code = int(result) if isinstance(result, int) else 0
                except SystemExit as exc:
                    code = int(exc.code or 0)
            finally:
                sys.argv = old_argv
            payload = {"request_id": request_id, "return_code": code}
        except BaseException as exc:
            traceback.print_exc()
            payload = {
                "request_id": str(request.get("request_id", "")) if isinstance(request, dict) else "",
                "return_code": 1,
                "error": f"{type(exc).__name__}: {exc}",
            }
        print(RESULT_PREFIX + json.dumps(payload, ensure_ascii=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
