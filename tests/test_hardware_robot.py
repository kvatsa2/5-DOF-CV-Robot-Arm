"""
HardwareRobot + PickPipeline with the MATLAB server and servo bus faked out.

The pipeline tests use SimRobot, which never fails. These run the REAL
HardwareRobot.send_target_pose / set_gripper logic (floor guard, IK error
handling, servo error handling) so a failure partway through a pick is shown
to stop the sequence on the backend that actually drives the arm.
"""

from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from vision_pipeline import config
from vision_pipeline.pipeline import PickAbortedError, PickPipeline
from vision_pipeline.planning.pick import PickTarget
from vision_pipeline.robot_interface.base import Pose
from vision_pipeline.robot_interface.hardware import HardwareRobot
from vision_pipeline.robot_interface.matlab_client import IKUnreachableError
from vision_pipeline.robot_interface.servo_driver import ServoSafetyError


class _FakeIK:
    """Stands in for MatlabIKClient. Fails the call numbered in `fail_on` (1-based)."""

    def __init__(self, fail_on=None):
        self.fail_on = fail_on
        self.calls = 0

    def request_ik(self, x, y, z, seed_rad=None):
        self.calls += 1
        if self.calls == self.fail_on:
            raise IKUnreachableError("target outside workspace")
        return [0.1, 0.2, 0.3, 0.4, 0.5], 0.5

    def request_fk(self, angles_rad):
        return np.eye(4)

    def close(self):
        pass


class _FakeBus:
    """Stands in for ServoBus: records moves, optionally refuses the gripper."""

    def __init__(self, gripper_refuses=False):
        self.moves = []
        self.gripper_moves = []
        self.gripper_refuses = gripper_refuses

    def rad_to_ticks(self, servo_id, rad):
        return int(2048 + 600 * rad)

    def ticks_to_rad(self, servo_id, ticks):
        return (ticks - 2048) / 600

    def read_position(self, servo_id):
        return 2048

    def move_joints_stepped(self, targets):
        self.moves.append(dict(targets))
        return dict(targets)

    def move_and_verify(self, servo_id, target_tick):
        if self.gripper_refuses:
            raise ServoSafetyError("gripper move refused")
        self.gripper_moves.append(target_tick)
        return target_tick

    def close(self):
        pass


def _robot(ik, bus) -> HardwareRobot:
    # Skip __init__: it opens a TCP socket and a serial port.
    robot = HardwareRobot.__new__(HardwareRobot)
    robot.matlab_client = ik
    robot.servo_bus = bus
    robot._last_angles_rad = [0.0] * 5
    return robot


def _target() -> PickTarget:
    return PickTarget(
        x=0.15, y=0.0, z=config.TABLE_Z_IN_BASE + config.PICK_Z_OFFSET, yaw_deg=0.0, num_studs=6
    )


def test_full_pick_succeeds_on_hardware_backend():
    bus = _FakeBus()
    PickPipeline(robot=_robot(_FakeIK(), bus)).execute_pick(_target())
    assert len(bus.moves) == 4, "hover, descend, (grasp), lift"
    assert len(bus.gripper_moves) == 2, "open at hover, close at grasp"


def test_unreachable_hover_stops_before_anything_moves():
    bus = _FakeBus()
    with pytest.raises(PickAbortedError, match="step 1"):
        PickPipeline(robot=_robot(_FakeIK(fail_on=1), bus)).execute_pick(_target())
    assert bus.moves == []
    assert bus.gripper_moves == [], "gripper must not open after a refused hover"


def test_unreachable_descend_stops_before_gripper_closes():
    bus = _FakeBus()
    with pytest.raises(PickAbortedError, match="step 2"):
        PickPipeline(robot=_robot(_FakeIK(fail_on=2), bus)).execute_pick(_target())
    assert len(bus.moves) == 1
    assert len(bus.gripper_moves) == 1, "only the open at hover; never closed"


def test_refused_gripper_stops_the_pick():
    bus = _FakeBus(gripper_refuses=True)
    with pytest.raises(PickAbortedError, match="gripper open failed"):
        PickPipeline(robot=_robot(_FakeIK(), bus)).execute_pick(_target())
    assert len(bus.moves) == 1, "no descend after the gripper failed to open"


def test_floor_guard_refusal_is_reported_as_false():
    robot = _robot(_FakeIK(), _FakeBus())
    below_floor = Pose(x=0.15, y=0.0, z=config.TABLE_Z_IN_BASE)
    assert robot.send_target_pose(below_floor) is False
