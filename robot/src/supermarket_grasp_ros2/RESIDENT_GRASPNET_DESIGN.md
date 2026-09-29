# Resident GraspNet design and parameter-isolation contract

## Runtime architecture

`grasp_node` remains the ROS2-facing process.  It keeps the existing trigger
services, parameter service, sensor caches, output topics, and FSM contract.
It starts one `grasp_resident_worker.py` process.  The worker imports
`grasp.py` once and receives one complete argument vector per request.  The
GraspNet checkpoint is cached by absolute path, mtime, device, and `num_view`.

The worker is deliberately serialized: only one request is sent at a time,
so a left and right request cannot mutate a shared `sys.argv` or model context
at the same time.  Every response carries a request ID.

## Isolation rules

- `arm`, `box`, `angle`, `approach`, and all geometry flags are sent on every
  request; no previous request value is used as a default.
- RGB-D frame bundle, bbox, diagnostics, confirmation, and output paths contain
  a UUID and cannot be mistaken for a previous request's files.
- Candidate arrays, cylinder geometry, transforms, joint seeds, and detection
  data remain local to one `grasp.py` invocation.  Only the read-only model is
  resident.
- `left_extra_args` and `right_extra_args` remain separate ROS parameters.
  The generic `extra_args` parameter is retained for compatibility.
- Interactive confirmation is also serialized through the resident worker;
  the ROS node forwards its status line and reads the UUID-scoped confirmation
  file after the request completes.
- If the worker exits or returns a protocol error, the next trigger recreates
  it.  `resident_mode:=false` restores the old subprocess behavior.

## Rollback

Launch with `resident_mode:=false` to disable resident execution without
changing any FSM or task configuration.
