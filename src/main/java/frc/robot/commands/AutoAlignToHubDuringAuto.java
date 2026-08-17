// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter; 
import frc.robot.util.SOTMUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignToHubDuringAuto extends Command {
  String name = "DriveWhileAiming";
  Drive drive;
  Shooter shooter;
  DoubleSupplier xSupplier;
  DoubleSupplier ySupplier;
  boolean sotm;
  double omega;
  PIDController fieldPIDController = new PIDController(0.12 - 0.04, 0.0, 0);
  boolean oneCycleDone = false;
        


  /** Creates a new AutoAlignToHubDuringAuto. */
  public AutoAlignToHubDuringAuto(Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Shooter shooter,
      boolean sotm) {
    this.drive = drive;
    this.shooter = shooter;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.sotm = sotm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fieldPIDController.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d targetPose = Constants.FieldConstants.getAllianceHubPose();
    double distance = drive.getPose().getTranslation().getDistance(targetPose.getTranslation());
    double lastSetpoint = 0;
    double setpoint = shooter.getVelo(distance);
    double time = 0;
    while (setpoint - lastSetpoint > 0.5) {
      time = SOTMUtil.timeOfFlight(distance);
      distance = SOTMUtil.getVirtualGoalDistance(
          time,
          drive.getPose(),
          drive.getFieldRelativeChassisSpeeds(),
          FieldConstants.getAllianceHubPose());
      lastSetpoint = setpoint;
      setpoint = shooter.getVelo(distance);
    }
    targetPose = sotm
        ? SOTMUtil.getVirtualGoalPose(
            time,
            drive.getPose(),
            drive.getFieldRelativeChassisSpeeds(),
            FieldConstants.getAllianceHubPose())
        : FieldConstants.getAllianceHubPose();
    Pose2d displacement = drive.getPose().relativeTo(targetPose);

    // Get linear velocity
    Translation2d linearVelocity = DriveCommands.getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
        ySupplier.getAsDouble());

    // Chnage aiming position based on current velocities
    // Transform2d onTheMove = new Transform2d(linearVelocity.getX()*.01,
    // linearVelocity.getY()*.01, Rotation2d.kZero);
    // targetPose.transformBy(onTheMove);
    // Logger.recordOutput(name + "/distance",
    //     drive.getPose().getTranslation().getDistance(targetPose.getTranslation()));
    // Logger.recordOutput(name + "/targetPose", targetPose);
    // Logger.recordOutput(name + "/displacementThetaDeg",
    //     Math.toDegrees(Math.atan2(displacement.getY(), displacement.getX())));

    // Apply rotation deadband
    boolean isFlipped = DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;

    omega = 0.0;
    double currentHeading = DriveCommands.getDriveHeading(drive);
    double targetAngle = DriveCommands.getRightStickAngle(() -> displacement.getY(), () -> displacement.getX())
        + (isFlipped ? 0 : 0);
    boolean rotationEnabled = Math.abs(displacement.getX() + displacement.getY()) > 0.1;

    if (rotationEnabled) {
      omega = fieldPIDController.calculate(currentHeading, targetAngle);
    }

    // Logger.recordOutput(name + "/currentHeading", currentHeading);
    // Logger.recordOutput(name + "/targetAngle", targetAngle);
    // Logger.recordOutput(name + "/headingError", targetAngle - currentHeading);
    // Logger.recordOutput(name + "/omega", omega);
    // Logger.recordOutput(name + "/displacementX", displacement.getX());
    // Logger.recordOutput(name + "/displacementY", displacement.getY());
    // Logger.recordOutput(name + "/rotationEnabled", rotationEnabled);
    // Logger.recordOutput(name + "/isFlipped", isFlipped);

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds = new ChassisSpeeds(
        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
        omega);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped
                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                : drive.getRotation()));
                oneCycleDone = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(0, 0, 0),
            Rotation2d.kZero));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
