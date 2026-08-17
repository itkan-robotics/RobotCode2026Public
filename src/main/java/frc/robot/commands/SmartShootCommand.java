// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.util.SOTMUtil;

public class SmartShootCommand extends Command {

  private final Shooter shooter;
  private final Transfer transfer;
  private final Conveyor conveyor;
  private final Drive drive;

  private double setpoint;
  private double distance;
  private Pose2d targetPose;

  public double tolerance = 0.5;

  // Below this fraction of target RPS, transfer/conveyor output is zero.
  // Above 1.0, output is 100%. Linear interpolation in between.
  private static final double MIN_VELOCITY_FRACTION = 0.90;

  public SmartShootCommand(Drive drive, Shooter shooter, Transfer transfer, Conveyor conveyor) {
    this.shooter = shooter;
    this.transfer = transfer;
    this.conveyor = conveyor;
    this.drive = drive;
    addRequirements(shooter, transfer, conveyor);
  }

  @Override
  public void initialize() {
    transfer.feeding = false;
    transfer.feedTimer.reset();
  }

  @Override
  public void execute() {
    targetPose = Constants.FieldConstants.getAllianceHubPose();
    SOTMUtil.ShotSolution shot = SOTMUtil.computeConvergedShot(
        drive.getPose(),
        drive.getFieldRelativeChassisSpeeds(),
        targetPose,
        shooter::getVelo);
    distance = shot.distance();
    setpoint = shot.setpoint();

    Logger.recordOutput(getName() + "/targetPose", targetPose);
    shooter.setBangBangSetpoint(setpoint - 0.4);

    Logger.recordOutput(getName() + "/distance", distance);
    Logger.recordOutput(getName() + "/setpoint", setpoint);

    // Use average of both shooter sides for velocity ratio
    double currentVelocity =
        (shooter.getLeftShooterVelocity() + shooter.getRightShooterVelocity()) / 2.0;

    double feedPower = computeFeedPower(currentVelocity, setpoint);

    Logger.recordOutput(getName() + "/feedPower", feedPower);
    Logger.recordOutput(getName() + "/velocityRatio", setpoint > 0 ? currentVelocity / setpoint : 0);

    //If either shooter velocity is greater than the lower bound of the tolerance or feeding
    if (shooter.getLeftShooterVelocity() > setpoint-tolerance
        || shooter.getRightShooterVelocity() > setpoint-tolerance || transfer.feeding) {
      transfer.hasPreloaded = false;
      transfer.feedTimer.start();
      transfer.feeding = true;
      transfer.setFeedPower(feedPower);
      if (drive.aligned) {
        transfer.setTransferDutyCycle(feedPower); //feedPower
        conveyor.setLeftFunnelDutyCycle(-1.0);
      } else {
        transfer.setTransferDutyCycle(0);
        conveyor.setLeftFunnelDutyCycle(-1.0);
      }
    } else {
      transfer.setFeedPower(0);
    }
  }

  private double computeFeedPower(double currentVelocity, double targetVelocity) {
    if (targetVelocity <= 0) {
      return 0;
    }
    double ratio = currentVelocity / targetVelocity;
    return MathUtil.clamp((ratio - MIN_VELOCITY_FRACTION) / (1.0 - MIN_VELOCITY_FRACTION) * 2.5, 0.5, 1.0);
  }

  @Override
  public void end(boolean interrupted) {
    transfer.feeding = false;
    transfer.feedTimer.reset();
    transfer.setFeedPower(0);
    transfer.setTransferDutyCycle(0);
    conveyor.setLeftFunnelDutyCycle(0);
    shooter.setDutyCycleSetpoint(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
