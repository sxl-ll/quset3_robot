"""Validated, read-only Quest 3 state acquisition using XRoboToolkit.

The module deliberately keeps XR data in the SDK coordinate convention. It
does not perform pose mapping, inverse kinematics, or robot control.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Mapping, Sequence

import numpy as np

from xrobotoolkit_teleop.common.xr_client import XrClient


POSE_ORDER = "[x, y, z, qx, qy, qz, qw]"
QUATERNION_ORDER = "xyzw"
POSITION_UNIT = "unverified SDK unit (expected to be confirmed on the target Quest 3 setup)"
XR_COORDINATE_SYSTEM = (
    "XRoboToolkit raw left-handed frame: +X right, +Y up, +Z inward; "
    "origin documented at the headset pose when the XR application starts"
)

POSE_NAMES = ("headset", "left_controller", "right_controller")
ANALOG_INPUT_NAMES = ("left_grip", "right_grip", "left_trigger", "right_trigger")
BUTTON_NAMES = (
    "A",
    "B",
    "X",
    "Y",
    "left_menu_button",
    "right_menu_button",
    "left_axis_click",
    "right_axis_click",
)


class Quest3ReadError(RuntimeError):
    """Raised when a complete Quest 3 snapshot cannot be read from the SDK."""


@dataclass(frozen=True)
class PoseSample:
    """One raw XR pose and its local validity assessment."""

    raw: tuple[float, ...]
    position: tuple[float, float, float] | None
    orientation_xyzw: tuple[float, float, float, float] | None
    valid: bool
    issue: str | None = None


@dataclass(frozen=True)
class Quest3Snapshot:
    """One complete read of the headset, controllers, and controller inputs."""

    timestamp_ns: int
    host_monotonic_ns: int
    timestamp_advanced: bool
    timestamp_age_s: float
    stale: bool
    timestamp_valid: bool
    tracking_valid: bool
    poses: Mapping[str, PoseSample]
    analog_inputs: Mapping[str, float]
    buttons: Mapping[str, bool]
    joysticks: Mapping[str, tuple[float, float]]
    issues: tuple[str, ...]


class Quest3Reader:
    """Read and validate Quest 3 state through an existing :class:`XrClient`.

    XRoboToolkit exposes one global tracking timestamp but no controller/headset
    validity getter in the Python API used by this project. Consequently,
    tracking validity here is conservative and is inferred from timestamp
    freshness, finite seven-element poses, and unit-length quaternions.
    """

    def __init__(
        self,
        xr_client: XrClient,
        stale_timeout_s: float = 0.5,
        quaternion_norm_tolerance: float = 1e-2,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ):
        if stale_timeout_s <= 0:
            raise ValueError("stale_timeout_s must be positive")
        if quaternion_norm_tolerance <= 0:
            raise ValueError("quaternion_norm_tolerance must be positive")

        self.xr_client = xr_client
        self.stale_timeout_s = float(stale_timeout_s)
        self.quaternion_norm_tolerance = float(quaternion_norm_tolerance)
        self._monotonic_ns = monotonic_ns
        self._last_timestamp_ns: int | None = None
        self._last_timestamp_change_monotonic_ns = self._monotonic_ns()

    def _parse_pose(self, name: str, value: Sequence[float]) -> PoseSample:
        try:
            pose = np.asarray(value, dtype=np.float64).reshape(-1)
        except (TypeError, ValueError) as exc:
            return PoseSample((), None, None, False, f"{name}: pose is not numeric ({exc})")

        raw = tuple(float(item) for item in pose)
        if pose.shape != (7,):
            return PoseSample(raw, None, None, False, f"{name}: expected 7 pose values, got {pose.size}")

        position = tuple(float(item) for item in pose[:3])
        quaternion = tuple(float(item) for item in pose[3:])
        if not np.all(np.isfinite(pose)):
            return PoseSample(raw, position, quaternion, False, f"{name}: pose contains non-finite values")

        quaternion_norm = float(np.linalg.norm(pose[3:]))
        if not math.isclose(quaternion_norm, 1.0, abs_tol=self.quaternion_norm_tolerance):
            return PoseSample(
                raw,
                position,
                quaternion,
                False,
                f"{name}: invalid quaternion norm {quaternion_norm:.6f}",
            )

        return PoseSample(raw, position, quaternion, True)

    @staticmethod
    def _parse_axis(name: str, value: Sequence[float]) -> tuple[tuple[float, float], str | None]:
        try:
            axis = np.asarray(value, dtype=np.float64).reshape(-1)
        except (TypeError, ValueError) as exc:
            return (math.nan, math.nan), f"{name}: axis is not numeric ({exc})"
        if axis.shape != (2,) or not np.all(np.isfinite(axis)):
            return (math.nan, math.nan), f"{name}: expected two finite axis values"
        return (float(axis[0]), float(axis[1])), None

    def _update_timestamp_state(self, timestamp_ns: int, now_ns: int) -> tuple[bool, float, bool, str | None]:
        timestamp_advanced = False
        timestamp_issue = None

        if timestamp_ns <= 0:
            timestamp_issue = "tracking timestamp is zero or negative; no active tracking data is available"
        elif self._last_timestamp_ns is None:
            timestamp_advanced = True
            self._last_timestamp_ns = timestamp_ns
            self._last_timestamp_change_monotonic_ns = now_ns
        elif timestamp_ns > self._last_timestamp_ns:
            timestamp_advanced = True
            self._last_timestamp_ns = timestamp_ns
            self._last_timestamp_change_monotonic_ns = now_ns
        elif timestamp_ns < self._last_timestamp_ns:
            timestamp_issue = (
                f"tracking timestamp regressed from {self._last_timestamp_ns} to {timestamp_ns}"
            )
            self._last_timestamp_ns = timestamp_ns
            self._last_timestamp_change_monotonic_ns = now_ns

        timestamp_age_s = max(0.0, (now_ns - self._last_timestamp_change_monotonic_ns) / 1e9)
        stale = timestamp_ns <= 0 or timestamp_age_s > self.stale_timeout_s
        if stale and timestamp_ns > 0 and timestamp_issue is None:
            timestamp_issue = (
                f"tracking timestamp has not advanced for {timestamp_age_s:.3f}s "
                f"(limit {self.stale_timeout_s:.3f}s)"
            )
        return timestamp_advanced, timestamp_age_s, stale, timestamp_issue

    def read(self) -> Quest3Snapshot:
        """Read one complete raw snapshot and attach freshness/validity status."""
        try:
            timestamp_before = int(self.xr_client.get_timestamp_ns())
            raw_poses = {name: self.xr_client.get_pose_by_name(name) for name in POSE_NAMES}
            analog_inputs = {
                name: float(self.xr_client.get_key_value_by_name(name)) for name in ANALOG_INPUT_NAMES
            }
            buttons = {name: bool(self.xr_client.get_button_state_by_name(name)) for name in BUTTON_NAMES}
            left_axis_raw = self.xr_client.get_joystick_state("left")
            right_axis_raw = self.xr_client.get_joystick_state("right")
            timestamp_ns = int(self.xr_client.get_timestamp_ns())
        except Exception as exc:
            raise Quest3ReadError(f"XRoboToolkit snapshot read failed: {exc}") from exc

        now_ns = self._monotonic_ns()
        issues: list[str] = []
        poses = {name: self._parse_pose(name, raw_poses[name]) for name in POSE_NAMES}
        issues.extend(sample.issue for sample in poses.values() if sample.issue is not None)

        for name, value in analog_inputs.items():
            if not math.isfinite(value):
                issues.append(f"{name}: input is not finite")

        left_axis, left_axis_issue = self._parse_axis("left_axis", left_axis_raw)
        right_axis, right_axis_issue = self._parse_axis("right_axis", right_axis_raw)
        if left_axis_issue:
            issues.append(left_axis_issue)
        if right_axis_issue:
            issues.append(right_axis_issue)

        if timestamp_before != timestamp_ns:
            issues.append(
                "tracking timestamp changed while reading separate SDK fields; "
                "this snapshot may span two XR updates"
            )

        timestamp_advanced, timestamp_age_s, stale, timestamp_issue = self._update_timestamp_state(
            timestamp_ns, now_ns
        )
        if timestamp_issue:
            issues.append(timestamp_issue)

        timestamp_valid = timestamp_ns > 0 and not stale and timestamp_issue is None
        tracking_valid = (
            timestamp_valid
            and all(sample.valid for sample in poses.values())
            and all(math.isfinite(value) for value in analog_inputs.values())
            and left_axis_issue is None
            and right_axis_issue is None
        )

        return Quest3Snapshot(
            timestamp_ns=timestamp_ns,
            host_monotonic_ns=now_ns,
            timestamp_advanced=timestamp_advanced,
            timestamp_age_s=timestamp_age_s,
            stale=stale,
            timestamp_valid=timestamp_valid,
            tracking_valid=tracking_valid,
            poses=poses,
            analog_inputs=analog_inputs,
            buttons=buttons,
            joysticks={"left": left_axis, "right": right_axis},
            issues=tuple(issues),
        )


def format_snapshot(snapshot: Quest3Snapshot) -> str:
    """Format a snapshot for the standalone console monitor."""
    freshness = "advanced" if snapshot.timestamp_advanced else "unchanged"
    status = "VALID" if snapshot.tracking_valid else "INVALID"
    lines = [
        (
            f"timestamp_ns={snapshot.timestamp_ns} timestamp={freshness} "
            f"age={snapshot.timestamp_age_s:.3f}s tracking={status}"
        )
    ]
    for name in POSE_NAMES:
        sample = snapshot.poses[name]
        if sample.position is None or sample.orientation_xyzw is None:
            lines.append(f"  {name:16s} INVALID raw={sample.raw}")
            continue
        pos = ", ".join(f"{value:+.5f}" for value in sample.position)
        quat = ", ".join(f"{value:+.5f}" for value in sample.orientation_xyzw)
        pose_status = "ok" if sample.valid else "INVALID"
        lines.append(f"  {name:16s} pos=({pos}) quat_xyzw=({quat}) [{pose_status}]")

    analog = " ".join(f"{name}={value:.3f}" for name, value in snapshot.analog_inputs.items())
    buttons = " ".join(f"{name}={int(value)}" for name, value in snapshot.buttons.items())
    axes = " ".join(f"{name}=({value[0]:+.3f},{value[1]:+.3f})" for name, value in snapshot.joysticks.items())
    lines.extend((f"  analog: {analog}", f"  buttons: {buttons}", f"  axes: {axes}"))
    lines.extend(f"  warning: {issue}" for issue in snapshot.issues)
    return "\n".join(lines)
