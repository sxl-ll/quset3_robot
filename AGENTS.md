# Quest 3 Single-Arm Teleoperation Project

## 1. Project Goal

Build a lightweight Quest 3 teleoperation and inverse kinematics system for a **single Songling robotic arm**, based on selected reusable components of the original XRoboToolkit teleoperation project.

The current objective is to obtain the Quest 3 controller pose, map it to a target robot end-effector pose, solve inverse kinematics using the existing project's IK implementation, and output the resulting joint angles.

**Target architecture:**

```text
Meta Quest 3
    |
    v
XRoboToolkit PC Service
    |
    v
XRoboToolkit Python SDK
    |
    v
Controller Pose Acquisition
    |
    v
Relative Cartesian Pose Mapping
    |
    v
Target End-Effector Pose
    |
    v
Existing Project IK Solver
    |
    v
Joint Angle Output
```

**Current system boundary:**

- One Songling robotic arm only.
- No ROS or MoveIt integration.
- No physical robot actuation.
- No automatic sending of joint commands.
- IK results are calculated, validated, and displayed only.

The exact robot model, degrees of freedom, joint limits, URDF, and coordinate conventions must be verified from project files or provided robot specifications. Do not invent robot parameters.

## 2. Development Principles

### 2.1 Code Reuse

1. Read the existing implementation before making changes.
2. Prefer reusing the original project's proven IK methods.
3. Do not introduce a new IK framework unless explicitly requested.
4. Preserve existing mathematical methods and verified algorithms.
5. Adapt existing modules to the Songling robotic arm rather than rewriting the complete system.
6. Do not assume that unused-looking files are unnecessary.
7. Trace imports, configuration references, runtime dependencies, and robot assets before removing files.

### 2.2 Project Scope

The project must focus on:

- Quest 3 headset and controller tracking.
- Controller input acquisition.
- Relative Cartesian pose mapping.
- Single-arm end-effector target generation.
- Existing IK solver integration.
- Robot joint-angle calculation.
- Joint limit and IK validity checking.
- Debugging, logging, visualization, and tests where useful.

Do not implement unrelated features.

### 2.3 Code Modification Rules

1. Work on one subsystem at a time.
2. Analyze before modifying.
3. Prefer small, maintainable changes.
4. Avoid unnecessary abstraction.
5. Do not duplicate existing functionality.
6. Keep unrelated modifications out of each task.
7. Preserve original LICENSE files and required copyright notices.
8. Do not modify the upstream reference repository.
9. Do not modify Git remotes, create commits, or push automatically.
10. Do not delete files without first providing a deletion proposal.
11. Verify changes using syntax checks, import checks, and relevant tests.
12. Clearly distinguish verified facts, assumptions, and unknown information.
13. When changing existing behavior, explain the reason and expected consequences.

## 3. Hardware Configuration

### 3.1 XR Input Device

Use Meta Quest 3 with XRoboToolkit.

Required inputs:

- Headset position and orientation.
- Left controller position and orientation.
- Right controller position and orientation.
- Grip and Trigger values.
- Controller button states.
- Tracking validity and timestamps where available.

Prioritize right-controller teleoperation initially.

Do not implement additional VR device integrations.

### 3.2 Robot Platform

The target robot is a **single Songling robotic arm**.

Requirements:

- Support only the intended Songling robot model.
- Use the correct robot kinematic configuration.
- Load the required robot model and joint parameters.
- Preserve necessary URDF, mesh, and configuration files.
- Verify joint ordering, units, joint limits, and reference frames.
- Do not assume a specific number of joints before checking the model.

Do not retain other robot drivers or configurations unless they are required dependencies of reusable IK components.

## 4. XR Data Acquisition

Use the existing XRoboToolkit Python SDK whenever possible.

Required functionality:

1. Initialize the SDK.
2. Read Quest 3 controller poses.
3. Read headset pose.
4. Read controller button states.
5. Check tracking validity.
6. Detect stale or interrupted tracking data.
7. Print or log the acquired data.
8. Shut down the SDK cleanly.

Use actual SDK APIs verified from the installed package or repository.

Do not invent SDK function names or data structures.

Preserve raw device data where necessary for debugging.

## 5. Pose Mapping

Implement relative Cartesian pose mapping between the Quest 3 controller and the robot end effector.

Required functionality:

- Record the initial controller pose.
- Record the initial robot end-effector pose.
- Calculate controller position and orientation changes.
- Transform those changes into the robot base frame.
- Calculate the target end-effector pose.
- Support configurable position scaling.
- Support a Grip-based clutch mechanism.
- Avoid discontinuities when teleoperation is activated or deactivated.
- Prevent invalid tracking data from updating the target.

Coordinate transformation requirements:

1. Identify the XR coordinate convention.
2. Identify the robot base coordinate convention.
3. Verify quaternion ordering.
4. Verify position units.
5. Apply proper rigid-body rotations.
6. Do not mix local-frame and world-frame transformations.

The pose mapping must be testable independently of the IK solver.

## 6. Inverse Kinematics

**Use the IK solving method already implemented in the original XRoboToolkit teleoperation project.**

Do not replace it with another IK framework without explicit approval.

### 6.1 Required IK Functionality

- Receive a target end-effector pose.
- Load or initialize the Songling robot kinematic model.
- Calculate the corresponding robot joint configuration.
- Return the joint angles in the correct joint order.
- Detect unsuccessful IK solutions.
- Check joint limits.
- Handle unreachable target poses.
- Support repeated solutions for continuously changing targets.

### 6.2 IK Implementation Requirements

1. Identify the existing IK implementation and its dependencies.
2. Understand its target frame and solver inputs.
3. Determine how to load the Songling robot model.
4. Reuse the existing solver and necessary utilities.
5. Avoid modifying the core solver unless required for compatibility or correctness.
6. Document any required modifications.
7. Do not silently substitute a different solver.
8. Preserve required kinematic libraries, configuration files, and robot assets.

### 6.3 IK Validation

Where supported by the project, verify:

- Joint-angle output dimension.
- Joint ordering.
- Joint-angle units.
- Joint limits.
- Forward-kinematics reconstruction error.
- Position and orientation errors.
- Solution continuity.
- Unreachable pose behavior.
- Numerical stability near singular configurations.

The exact acceptance thresholds must be based on the solver capabilities and robot specifications.

Do not report an IK solution as valid solely because the solver returned a numerical vector.

## 7. Current Output Requirements

The current project only needs to calculate and output robot joint angles.

Expected processing flow:

```text
Quest Controller Pose
        |
        v
Pose Mapping
        |
        v
Target End-Effector Pose
        |
        v
Existing IK Solver
        |
        v
Joint Angle Vector
        |
        v
Validation and Console Output
```

Output should include:

- Target end-effector pose.
- IK success or failure status.
- Calculated joint angles.
- Joint-angle units.
- Joint limit validation results.
- Position and orientation error when available.

Do not send the calculated joint angles to the physical robot.

The system should be designed so that a real robot controller can be integrated later without rewriting the XR input, pose mapping, or IK modules.

## 8. Out of Scope

The following features are not required at the current stage:

- ROS 1.
- ROS 2.
- MoveIt.
- Physical robot motion commands.
- Real robot joint controllers.
- Multi-robot teleoperation.
- Dual-arm coordination.
- Other robot hardware drivers.
- Dexterous hand control.
- Camera streaming.
- Video recording.
- Imitation-learning dataset recording.
- Reinforcement learning.
- Automatic execution of hardware demonstrations.

Do not retain or implement these features unless they are necessary dependencies or explicitly requested.

## 9. Repository Simplification

The original XRoboToolkit repository contains functionality for several robots and different teleoperation scenarios.

The goal is to simplify the project while preserving the components necessary for Quest 3 teleoperation and Songling single-arm IK calculation.

### Phase 1: Repository Analysis

Before deleting anything:

1. Inspect the repository structure.
2. Identify the XR data acquisition modules.
3. Identify the pose mapping modules.
4. Identify the existing IK solver.
5. Trace required imports and runtime dependencies.
6. Identify necessary robot models and configuration files.
7. Classify files as:
   - Required.
   - Potentially required.
   - Unrelated.
8. Produce a proposed minimal project structure.

Do not modify or delete files during this phase.

### Phase 2: Minimal Functional Pipeline

Build and verify:

1. Quest 3 data reading.
2. Controller pose mapping.
3. Songling robot kinematic model loading.
4. IK calculation.
5. Joint-angle output.

Test each subsystem independently before integrating them.

### Phase 3: Code Simplification

Remove unrelated modules incrementally.

Rules:

- Provide a deletion list before removing files.
- Explain why each proposed deletion is safe.
- Check static imports and dynamic dependencies.
- Check configuration and resource references.
- Remove only one logical group at a time.
- Re-run relevant tests after each modification.
- Preserve working implementations until replacements are validated.

Do not perform large-scale automatic deletion.

## 10. Testing Requirements

Prioritize reproducible tests.

### XR Input Tests

- Controller pose updates correctly.
- Headset pose updates correctly.
- Grip and Trigger states update.
- Missing or stale data is detected.

### Pose Mapping Tests

- Zero controller displacement produces zero mapped displacement.
- Translation direction is correct.
- Orientation mapping is correct.
- Scaling behaves as configured.
- Clutch reactivation does not produce target jumps.

### IK Tests

- Valid poses produce valid joint configurations.
- Joint limits are respected.
- FK of calculated joints approximates the target pose within justified tolerances.
- Unreachable targets are handled safely.
- Repeated solutions do not exhibit unexplained discontinuities.

### Regression Tests

After modifying existing code:

1. Run syntax checks.
2. Run import tests.
3. Run relevant unit tests.
4. Compare results against known-good behavior where available.
5. Report which tests passed, failed, or could not be executed.

Do not claim hardware validation if tests only ran in software.

## 11. Git Safety Rules

1. Do not automatically commit changes.
2. Do not automatically push to remote repositories.
3. Do not modify Git remotes.
4. Do not rewrite Git history.
5. Do not perform destructive Git operations without explicit authorization.
6. Review the diff before considering work complete.
7. Keep each modification logically focused.

## 12. Codex Workflow

For every substantial task:

**Step 1 — Analyze**

Read the relevant code, configuration, and dependencies.

**Step 2 — Propose**

Explain the intended changes, assumptions, and potential risks.

**Step 3 — Implement**

Make the smallest reasonable changes required for the task.

**Step 4 — Verify**

Execute applicable tests and inspect the results.

**Step 5 — Report**

Summarize:

- Files modified.
- Functionality added or removed.
- Tests executed.
- Verified results.
- Remaining problems.
- Recommended next steps.

Do not claim a feature is complete without sufficient verification.

## 13. Final Development Objective

The initial project will be considered functionally complete when:

1. Quest 3 headset and controller data can be read reliably.
2. Controller movement can be mapped to a target end-effector pose.
3. The existing project IK method can solve the target pose using the Songling arm model.
4. Joint-angle vectors can be output correctly.
5. IK failures, joint limits, and discontinuities are handled appropriately.
6. The software can run without ROS or physical robot actuation.
7. Unrelated source code has been removed without breaking the verified functionality.

**The immediate priority is a reliable Quest 3 → Pose Mapping → Existing IK → Joint Angles pipeline, not real robot motion.**