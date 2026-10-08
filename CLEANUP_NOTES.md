# Cleanup notes (2026-10-07)

A cleanup pass on the `cleanup` branch was tuned for **reliability over tidiness**. Code that
works but looks rough was left alone when changing it carried real risk. This file records
those spots so they are known, not forgotten. See `git log main..cleanup` for what did change.

## Deliberately left alone

### MATLAB server listens on every network interface
[`matlab/ik_fk_server.m:39`](matlab/ik_fk_server.m) — `java.net.ServerSocket(PORT)` accepts
connections from any machine on the network, even though it prints "Listening on localhost".
The obvious fix is
`java.net.ServerSocket(PORT, 50, java.net.InetAddress.getLoopbackAddress())`. It was tried,
then reverted, because it could not be tested without MATLAB. Why the risk is low:
- The server only does math (IK/FK). It never commands motion. The servos are on a separate
  serial port that only the Python side opens.
- The Windows firewall normally blocks inbound connections from other machines.

To apply the fix, make the one-line change, start the server, and then run
`python scripts/test_matlab_bridge.py`.

### Broad `except Exception` in the hardware code
[`hardware.py`](src/vision_pipeline/robot_interface/hardware.py) (lines 70, 90, 144, 172, 193,
227) and [`servo_driver.py`](src/vision_pipeline/robot_interface/servo_driver.py) (428, 490)
catch every error. Linters flag this, but it is intentional:
- `ServoBus.freeze` must keep freezing the other joints when one servo fails.
- `read_diagnostics` is meant to give a partial answer about a servo that is already failing.
- `HardwareRobot` turns any failure into a logged `False`, which `PickPipeline` now treats as
  "stop the pick".

Narrowing these would risk letting an unexpected error type escape mid-move.

### Large files
`servo_driver.py` (~800 lines) and `config.py` (~650 lines, mostly explanatory comments) each
cover a single job. Splitting them would touch byte-level protocol code that has only been
tested against a fake serial port and a single real arm, for no change in behaviour.

### Remaining type-checker warnings (16)
`mypy` reports 16 warnings. None of them is a bug, and there were 23 before the cleanup. They
come from:
- 9 from the "serial port or socket may be `None` before connect" pattern in
  `servo_driver.py` and `matlab_client.py`
- 3 from `count_studs` returning `int` or `(int, circles)` depending on a flag
  (`lego_detector.py`)
- 4 one-offs that are all correct at runtime: OpenCV's stub for `matchImagePoints`, the
  `boundingRect` tuple in `color_detector.py`, the distortion tuple length in
  `camera_model.py`, and `camera_stream.py` passing an index that is never `None` on that
  path

The `count_studs` ones need `typing.overload` boilerplate to fix. The rest need casts or
asserts that only quiet the checker.

### Paths that still assume the repo root is the working directory
Every `data/...` path in `config.py` is relative, as is
[`generate_charuco_board.py:72`](scripts/generate_charuco_board.py) and the MATLAB path at
[`init_arm.m:104`](matlab/init_arm.m) (relative to `matlab/`). This works as long as scripts
run from the repo root and MATLAB runs from `matlab/`, which is how CLAUDE.md says to run
them. Changing to absolute paths would alter where existing calibration files are looked up.

### Script-local tuning constants
Scripts keep their own operator-facing constants: settle times, step sizes, clearance
margins (for example `jog_joint.py`'s `CLEARANCE_MARGIN_MM`). Each one is specific to one
procedure. Moving them into `config.py` would make that file harder to read, and no code
shares them.

## Known issues not caused by this cleanup
- `data/hand_eye.json` is about 52 mm wrong, and J5's `dir_sign` is probably inverted. See
  CLAUDE.md. Both need the physical arm to settle.
- No linter is configured. `ruff`, `mypy` and `vulture` were used for the audit, but they are
  not in `requirements.txt`.
