"""Load and validate the repository's Piper URDF with Placo."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import placo

from xrobotoolkit_teleop.utils.path_utils import PIPER_ASSET_DIR, PIPER_URDF_PATH


PIPER_PACKAGE_URI_PREFIX = "package://agx_arm_description/agx_arm_urdf/piper/"
PIPER_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
PIPER_END_EFFECTOR_FRAME = "link6"


class PiperModelError(RuntimeError):
    """Raised when the repository's Piper model is missing or inconsistent."""


def resolved_piper_urdf_content(
    urdf_path: Path = PIPER_URDF_PATH,
    asset_dir: Path = PIPER_ASSET_DIR,
) -> str:
    """Return URDF text with the known ROS package URI resolved locally.

    The official URDF is kept unchanged in ``assets``. Placo does not know the
    ROS package search path in this ROS-free project, so its mesh references
    are converted to absolute ``file://`` URIs in memory before loading.
    """
    urdf_path = Path(urdf_path).resolve()
    asset_dir = Path(asset_dir).resolve()
    if not urdf_path.is_file():
        raise PiperModelError(f"Piper URDF does not exist: {urdf_path}")
    if not asset_dir.is_dir():
        raise PiperModelError(f"Piper asset directory does not exist: {asset_dir}")

    urdf_content = urdf_path.read_text(encoding="utf-8")
    if PIPER_PACKAGE_URI_PREFIX not in urdf_content:
        raise PiperModelError(
            "Piper URDF does not contain the expected package URI prefix: "
            f"{PIPER_PACKAGE_URI_PREFIX}"
        )

    resolved_content = urdf_content.replace(
        PIPER_PACKAGE_URI_PREFIX,
        f"{asset_dir.as_uri()}/",
    )
    if "package://" in resolved_content:
        raise PiperModelError("Piper URDF contains an unresolved package:// resource")
    return resolved_content


def load_piper_robot() -> placo.RobotWrapper:
    """Load the Piper URDF and verify its expected kinematic identifiers."""
    urdf_content = resolved_piper_urdf_content()
    try:
        robot = placo.RobotWrapper(
            str(PIPER_ASSET_DIR),
            placo.Flags.ignore_collisions,
            urdf_content,
        )
    except Exception as exc:
        raise PiperModelError(f"Placo failed to load the Piper model: {exc}") from exc

    joint_names = tuple(robot.joint_names())
    if joint_names != PIPER_JOINT_NAMES:
        raise PiperModelError(
            f"Unexpected Piper joint order: {joint_names}; expected {PIPER_JOINT_NAMES}"
        )
    if PIPER_END_EFFECTOR_FRAME not in tuple(robot.frame_names()):
        raise PiperModelError(
            f"Piper end-effector frame is missing: {PIPER_END_EFFECTOR_FRAME}"
        )

    robot.update_kinematics()
    return robot


def piper_joint_limits(robot: placo.RobotWrapper) -> tuple[np.ndarray, np.ndarray]:
    """Return lower and upper joint limits in Piper joint order, in radians."""
    limits = np.asarray(
        [robot.get_joint_limits(name) for name in PIPER_JOINT_NAMES],
        dtype=np.float64,
    )
    if limits.shape != (len(PIPER_JOINT_NAMES), 2) or not np.all(np.isfinite(limits)):
        raise PiperModelError(f"Invalid Piper joint limits returned by Placo: {limits}")
    if np.any(limits[:, 0] > limits[:, 1]):
        raise PiperModelError(f"Piper joint lower limit exceeds upper limit: {limits}")
    return limits[:, 0].copy(), limits[:, 1].copy()


def piper_joint_positions(robot: placo.RobotWrapper) -> np.ndarray:
    """Return the six arm joint positions in verified Piper order, in radians."""
    positions = np.asarray(
        [robot.get_joint(name) for name in PIPER_JOINT_NAMES],
        dtype=np.float64,
    )
    if positions.shape != (len(PIPER_JOINT_NAMES),) or not np.all(np.isfinite(positions)):
        raise PiperModelError(f"Invalid Piper joint positions returned by Placo: {positions}")
    return positions


def set_piper_joint_positions(robot: placo.RobotWrapper, positions: np.ndarray) -> None:
    """Set the six Piper joint positions in verified order, in radians."""
    positions = np.asarray(positions, dtype=np.float64)
    if positions.shape != (len(PIPER_JOINT_NAMES),):
        raise ValueError(
            f"Expected {len(PIPER_JOINT_NAMES)} Piper joint positions, got shape {positions.shape}"
        )
    if not np.all(np.isfinite(positions)):
        raise ValueError("Piper joint positions must be finite")
    for name, position in zip(PIPER_JOINT_NAMES, positions):
        robot.set_joint(name, float(position))
    robot.update_kinematics()

