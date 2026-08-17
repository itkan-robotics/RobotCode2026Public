// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.SOTMUtil;
import frc.robot.util.Util;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 0.12;
  private static final double ANGLE_KD = 0.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private static PIDController fieldPIDController;

  private DriveCommands() {
  }

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  public static double getRightStickAngleDegrees(DoubleSupplier x, DoubleSupplier y) {
    double xx = MathUtil.applyDeadband(x.getAsDouble(), DEADBAND);
    double yy = MathUtil.applyDeadband(y.getAsDouble(), DEADBAND);
    return (Math.toDegrees(Math.atan2(xx, yy)));
  }

  public static double getRightStickAngle(DoubleSupplier x, DoubleSupplier y) {
    double xx = MathUtil.applyDeadband(x.getAsDouble(), DEADBAND);
    double yy = MathUtil.applyDeadband(y.getAsDouble(), DEADBAND);
    return (Math.toDegrees(Math.atan2(xx, yy)));
  }

  public static double getDriveHeading(Drive drive) {
    return MathUtil.inputModulus(drive.getRotation().getDegrees(), -180, 180);
  }

  public static double getDriveHeadingDegrees(Drive drive) {
    return MathUtil.inputModulus(drive.getRotation().getDegrees(), -180, 180);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and
   * angular velocities).
   */
  public static Command joystickMDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier jwxSupplier,
      DoubleSupplier jwySupplier) {

    fieldPIDController = new PIDController(ANGLE_KP - 0.04, 0.0, ANGLE_KD);
    fieldPIDController.enableContinuousInput(-180, 180);

    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(
              xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          // Apply rotation deadband
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;

          double omega = 0.0;

          if (Math.abs(jwxSupplier.getAsDouble() + jwySupplier.getAsDouble()) > 0.1) {
            omega = fieldPIDController.calculate(
                getDriveHeadingDegrees(drive),
                getRightStickAngleDegrees(jwxSupplier, jwySupplier)
                    + (isFlipped ? 0 : 180));

            // Logger.recordOutput("MDrive/error", fieldPIDController.getError());
            // Logger.recordOutput("MDrive/omega", omega);
          }

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega);

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(
                          new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>
   * This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
            () -> {
              drive.runCharacterization(0.0);
            },
            drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
            () -> {
              double voltage = timer.get() * FF_RAMP_RATE;
              drive.runCharacterization(voltage);
              velocitySamples.add(drive.getFFCharacterizationVelocity());
              voltageSamples.add(voltage);
            },
            drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i)
                        * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i)
                        * velocitySamples
                            .get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY)
                      / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY)
                      / (n * sumX2 - sumX * sumX);

                //   NumberFormat formatter = new DecimalFormat(
                //       "#0.00000");
                //   System.out.println(
                //       "********** Drive FF Characterization Results **********");
                //       SmartDashboard.putNumber("ks", kS);
                //       SmartDashboard.putNumber("kv", kV);
                //   System.out.println("\tkS: "
                //       + formatter.format(kS));
                //   System.out.println("\tkV: "
                //       + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */   //hiimabbad//
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter 
            
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(
                      WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0,
                      speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive
                      .getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                () -> {
                  var rotation = drive.getRotation();
                  state.gyroDelta += Math.abs(
                      rotation.minus(state.lastAngle)
                          .getRadians());
                  state.lastAngle = rotation;
                })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive
                          .getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(
                            positions[i] - state.positions[i])
                            / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta
                          * Drive.DRIVE_BASE_RADIUS)
                          / wheelDelta;

                      NumberFormat formatter = new DecimalFormat(
                          "#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: "
                              + formatter.format(
                                  wheelDelta)
                              + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter
                              .format(state.gyroDelta)
                              + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(
                                  wheelRadius)
                              + " meters, "
                              + formatter.format(
                                  Units.metersToInches(
                                      wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  public static Command allianceZoneSotmDriveCommand(
      Drive drive,
      Shooter shooter,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier jwxSupplier,
      DoubleSupplier jwySupplier) {

    String commandName = "AllianceZoneSOTM";
    return Commands.run(() -> {
      fieldPIDController = new PIDController(ANGLE_KP - 0.04, 0.0, ANGLE_KD);
      fieldPIDController.enableContinuousInput(-180, 180);

      Translation2d linearVelocity = getLinearVelocityFromJoysticks(
          xSupplier.getAsDouble(),
          ySupplier.getAsDouble());
      boolean isFlipped = isAllianceFlipped();
      Pose2d robotDisplacementFromTarget = getSotmDisplacementFromTarget(drive, shooter, false);

      double omega = calculateOmegaFromDriverOrTarget(
          drive,
          jwxSupplier,
          jwySupplier,
          () -> robotDisplacementFromTarget.getY(),
          () -> robotDisplacementFromTarget.getX(),
          isFlipped);

      ChassisSpeeds speeds = new ChassisSpeeds(
          linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
          linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
          omega);

      runVelocityOrStopX(drive, speeds, isFlipped, commandName, true);
    }, drive);
  }

  public static Command trenchAlignDriveCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier jwxSupplier,
      DoubleSupplier jwySupplier) {

    String commandName = "TrenchAlignCommand";
    return Commands.run(() -> {
      fieldPIDController = new PIDController(ANGLE_KP - 0.04, 0.0, ANGLE_KD);
      fieldPIDController.enableContinuousInput(-180, 180);

      Translation2d linearVelocity = getLinearVelocityFromJoysticks(
          xSupplier.getAsDouble(),
          ySupplier.getAsDouble());
      boolean isFlipped = isAllianceFlipped();

      double omega = 0.0;
      double addedYVelocity = 0.0;
      if (Math.abs(jwxSupplier.getAsDouble() + jwySupplier.getAsDouble()) > 0.1) {
        fieldPIDController.setP(ANGLE_KP - 0.04);
        fieldPIDController.setD(0.0);
        omega = fieldPIDController.calculate(
            getDriveHeadingDegrees(drive),
            getRightStickAngleDegrees(jwxSupplier, jwySupplier)
                + (isFlipped ? 0 : 180));
      } else {
        Translation2d trenchCenter = drive.getPose().getY() > 4
            ? new Translation2d(0.0, 7.500)
            : new Translation2d(0.0, 0.600);
        Util.getUtilField().getObject("leftTrench")
            .setPoses(new Pose2d[] { new Pose2d(0, 7.5, Rotation2d.kZero),
                new Pose2d(15, 7.5, Rotation2d.kZero) });
        Util.getUtilField().getObject("rightTrench")
            .setPoses(new Pose2d[] { new Pose2d(0, 0.6, Rotation2d.kZero),
                new Pose2d(15, 0.6, Rotation2d.kZero) });

        double yDisplacement = (drive.getPose().getY() - trenchCenter.getY())*(isFlipped ? 1 : -1);
        double targetAngle = Math.round(drive.getRotation().getDegrees() / 180) * 180;
        omega = fieldPIDController.calculate(getDriveHeadingDegrees(drive), targetAngle);
        addedYVelocity = yDisplacement * 0.25;
      }

      ChassisSpeeds speeds = new ChassisSpeeds(
          linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
          (linearVelocity.getY() + addedYVelocity)
              * drive.getMaxLinearSpeedMetersPerSec(),
          omega);

      runVelocityOrStopX(drive, speeds, isFlipped, commandName, false);
    }, drive);
  }

  public static boolean isAllianceFlipped() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }

  public static Pose2d getSotmDisplacementFromTarget(Drive drive, Shooter shooter, boolean passingTarget) {
    Pose2d originalTargetPose = passingTarget
        ? FieldConstants.getPassingPose(drive.getPose())
        : FieldConstants.getAllianceHubPose();
    double robotDistanceToTarget = drive.getPose().getTranslation()
        .getDistance(originalTargetPose.getTranslation());

    double timeOfFlight = 0;
    double robotHeading = drive.getPose().getRotation().getRadians();
    double robotNewHeading = Double.MAX_VALUE;
    Pose2d virtualTargetPose = originalTargetPose;
    while (Math.abs(robotHeading - robotNewHeading) > 0.1) {
      timeOfFlight = SOTMUtil.timeOfFlight(robotDistanceToTarget);
      virtualTargetPose = SOTMUtil.getVirtualGoalPose(
          timeOfFlight,
          drive.getPose(),
          drive.getFieldRelativeChassisSpeeds(),
          originalTargetPose);
      robotDistanceToTarget = drive.getPose().getTranslation()
          .getDistance(virtualTargetPose.getTranslation());
      robotHeading = robotNewHeading;
      robotNewHeading = Math.atan2(
          virtualTargetPose.getY() - originalTargetPose.getY(),
          virtualTargetPose.getX() - originalTargetPose.getX());
    }
    return drive.getPose().relativeTo(virtualTargetPose);
  }

  public static double calculateOmegaFromDriverOrTarget(
      Drive drive,
      DoubleSupplier jwxSupplier,
      DoubleSupplier jwySupplier,
      DoubleSupplier targetYSupplier,
      DoubleSupplier targetXSupplier,
      boolean isFlipped) {
    if (Math.abs(jwxSupplier.getAsDouble() + jwySupplier.getAsDouble()) > 0.1) {
      fieldPIDController.setP(ANGLE_KP - 0.04);
      fieldPIDController.setD(0.0);
      return fieldPIDController.calculate(
          getDriveHeadingDegrees(drive),
          getRightStickAngleDegrees(jwxSupplier, jwySupplier) + (isFlipped ? 0 : 180));
    }

    fieldPIDController.setP(ANGLE_KP * 0.9);
    fieldPIDController.setD(0.001);
    return fieldPIDController.calculate(
        getDriveHeadingDegrees(drive),
        getRightStickAngleDegrees(targetYSupplier, targetXSupplier));
  }

  private static void runVelocityOrStopX(
      Drive drive,
      ChassisSpeeds speeds,
      boolean isFlipped,
      String commandName,
      boolean useStopWithX) {
    if (useStopWithX
        && Math.abs(speeds.vxMetersPerSecond) + Math.abs(speeds.vyMetersPerSecond)
            + Math.abs(speeds.omegaRadiansPerSecond) < 0.3) {
      drive.stopWithX();
      Logger.recordOutput(commandName + "/stopWithX", true);
      drive.aligned = true;
      return;
    }
    drive.aligned = false;

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped
                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                : drive.getRotation()));
    Logger.recordOutput(commandName + "/stopWithX", false);
  }
}
