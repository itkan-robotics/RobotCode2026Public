FRC 9128 2026 Code: REBUILT
---

## What this robot does

On the field, the robot drives, collects game pieces from the floor, moves them through the robot, and launches them into the alliance hub. A front-mounted camera helps the robot know where it is, which makes autonomous paths and moving shots much more accurate.

| Subsystem | What it does |
|-----------|--------------|
| **Drive** | Moves and rotate in any direction (swerve drive) |
| **Intake** | Picks up game pieces from the floor and can deploy or retract |
| **Transfer + Conveyor** | Moves pieces through the robot toward the shooter |
| **Shooter** | Spins flywheels and launches pieces into the hub |
| **Vision** | Camera reads AprilTags on the field to correct the robot's position estimate |

## How the software is organized

The code is split into **subsystems** (the robot's mechanical parts in software) and **commands** (tasks like "shoot while driving" or "follow this path"). Everything is wired together in one central class, [`RobotContainer.java`](src/main/java/frc/robot/RobotContainer.java).

When the robot starts, it boots through a short chain of classes, sets up logging, creates all subsystems, registers button bindings, and loads autonomous routines.

---

## Swerve drive and pose estimation

Swerve drive means four independent wheel modules, each with its own steering angle and drive speed. The robot can move in any direction while rotating, which is essential for scoring quickly without stopping to turn.

The robot tracks **where it is on the field** using wheel encoders and a gyro (Pigeon 2). That estimate drifts over time, so a **Limelight** camera reads AprilTags and corrects position. The fused result feeds autonomous paths, hub aiming, and shoot-on-the-move (SOTM) calculations.


### Pose pipeline

[`Drive.java`](src/main/java/frc/robot/subsystems/drive/Drive.java) runs swerve kinematics, odometry, PathPlanner `AutoBuilder`, and SysId routines. [`Vision.java`](src/main/java/frc/robot/subsystems/vision/Vision.java) fuses Limelight MegaTag 1 and MegaTag 2 observations with ambiguity and Z-error filtering before calling `drive.addVisionMeasurement()`. Swerve geometry and gains come from CTRE-generated [`TunerConstants.java`](src/main/java/frc/robot/generated/TunerConstants.java) *(Note: regenerate from CTRE Tuner X, do not hand-edit)*.
>
### IO abstraction:

`ModuleIO`, `GyroIO`, and `VisionIO` interfaces let the same subsystem logic run in three modes. `RobotContainer` swaps implementations for REAL (TalonFX + Pigeon 2), SIM (`ModuleIOSim`), and REPLAY (no-op IO for log playback).

---

## Distance-based and shoot-on-the-move (SOTM) Shooting

Shooter flywheel speed depends on **distance to the hub**. A lookup table maps distance to rotations per second (RPS), so the robot can score from different positions on the field.

**Shoot-on-the-move (SOTM)** is harder: while the robot is driving, it must aim where the hub *will be* when the piece arrives, not where it is right now. The software models time of flight, adjusts the target based on current speed, and iterates until the flywheel setpoint converges.

| Mode | How it works |
|------|--------------|
| **Teleop smart shoot** | Hold **R1** and flywheel speed, feeding, agitation, and auto-aim drive run together |
| **Preset shots** | Triangle / Circle / Cross / Square trigger fixed-RPS shots for known distances |
| **Autonomous SOTM** | PathPlanner runs path segments while named commands `SOTM_Heading` and `SOTM_Shoot` fire in parallel |

### SOTM concept

[`SOTMUtil.java`](src/main/java/frc/robot/util/SOTMUtil.java) implements a quadratic time-of-flight model and iterative convergence via `computeConvergedShot()`. [`HubShiftUtil.java`](src/main/java/frc/robot/util/HubShiftUtil.java) tracks the 2026 alliance hub shift; the operator can override with L1/R1 on the operator controller. Key commands: [`SmartShootCommand`](src/main/java/frc/robot/commands/SmartShootCommand.java) (teleop), [`HubSOTMShotCommand`](src/main/java/frc/robot/commands/HubSOTMShotCommand.java) (auto), [`SotmHeadingOverrideCommand`](src/main/java/frc/robot/commands/SotmHeadingOverrideCommand.java) (overrides PathPlanner rotation PID during SOTM path segments). Field geometry and scoring zones live in [`Constants.FieldConstants`](src/main/java/frc/robot/Constants.java).

---

## Autonomous routines (PathPlanner)

Before each match, the drive team picks an auto from the dashboard. The robot follows pre-drawn paths on the field and, at specific moments, runs **named commands** (e.g.intake fuel, align to hub, shoot, SOTM, etc.).

The repo includes **35+ auto routines** and **85+ paths** under [`src/main/deploy/pathplanner/`](src/main/deploy/pathplanner/). Examples include standard left/right cycles (`1678T-L`, `1678T-R`), SOTM variants (`1678T-L-SOTM`, `1678T-L-90-SOTM`), steal routes, and tuning autos (`zTranslateTune`, `zRotateTune`).

### Example: `1678T-L-SOTM.auto`

This auto drives a cycle path, shoots on the move during a deadline group, stops the shooter, finishes the path, fires a stationary shot, and premoves toward the next position.

### Named commands available to autos

| Name | Purpose |
|------|---------|
| `SpinUp` | Spin flywheel to 43 RPS |
| `Intake_Fuel` | Deploy intake and run rollers |
| `Intake_Home` | Retract intake |
| `AlignToHub` | Drive while aiming at hub during auto |
| `Shoot_Fuel` / `Shoot_Fuel_Depot` | Distance-based shot (multipliers 1.5 / 2.0) |
| `Stop_Shoot` | Stop shooter, transfer, and conveyor |
| `SOTM_Heading` / `SOTM_Heading_Stop` | Enable / disable SOTM heading override on PathPlanner |
| `SOTM_Shoot` | Shoot-on-the-move with agitation |

Named commands are registered in `RobotContainer.registerNamedCommands()`. PathPlanner holonomic controller P gains are tunable via [`Constants.PathPlannerConstants`](src/main/java/frc/robot/Constants.java) and `LoggedTunableNumber` on the dashboard. Pathfinding uses [`LocalADStarAK`](src/main/java/frc/robot/util/LocalADStarAK.java), a replay-safe wrapper around PathPlanner's AD* pathfinder.

---

## AdvantageKit

Every sensor reading, pose update, and command decision can be recorded. After a match or practice run, replay the log through simulation to debug timing and tuning without being on the robot.

| Mode | When | What gets logged |
|------|------|------------------|
| **REAL** | Running on the roboRIO | WPILOG to USB (`/U/logs`) + NetworkTables (NT4) |
| **SIM** | Physics simulation on a laptop | NetworkTables only |
| **REPLAY** | Playing back a recorded log | Re-runs logic; writes a new `_sim` log |

---

## Teleop controls (quick reference)

| Input | Action |
|-------|--------|
| Left stick | Field-relative drive |
| Right stick | Manual heading |
| R2 | Deploy + intake |
| L2 | Outtake (transfer + conveyor + intake) |
| R1 | Smart shoot + SOTM drive + agitate |
| Triangle / Circle / Cross / Square | Fixed-RPS preset shots |
| Square (driver) | Trench align drive |
| POV Up / Down | Intake home / deploy |
| Operator L1 / R1 | Manual hub-shift override |
| Controller 5 (if connected) | Tuning: coast/brake/zero intake, PID shooter, manual feed |

We use PS5 controllers: driver (port 0), operator (port 1), optional test pad (port 5).
