// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.util.SOTMUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HubSOTMShotCommand extends Command {
  /** Creates a new RPMShootCommand. */
  // takes in the shooter the transfer conveyor intake and a rps
  // sets the shooter to the rps, once within threshold run transfer and coveyor
  // while running it bring the intake in halfway
  final Shooter shooter;
  final Transfer transfer;
  final Conveyor conveyor;
  final Drive drive;
  final BooleanSupplier speedBoostSupplier;
  public double rps;
  public double distance;
  double setpoint;
  Pose2d targetPose;
  final BooleanSupplier sotmSupplier;
  final double shooterSpeedTolerance = 6.5;

  // Cap for agitation(how far inwards pivot goes)
  private double TIMER_DELAY = 1;
  double multiplier = 3.5;
  private static final double SPEED_BOOST_FACTOR = 2.5;

  // tolerance in rps
  public double tolerance = 0.1;

  // hopper servos stuff
  private Timer hopperRampTimer = new Timer();
  private static final double hopperRampDuration = 3.0; // how long until reaching down position

  public HubSOTMShotCommand(Drive drive, Shooter shooter, Transfer transfer, Conveyor conveyor,
      BooleanSupplier speedBoostSupplier, BooleanSupplier sotmSupplier, DoubleSupplier multiplier) {
    this.shooter = shooter;
    this.transfer = transfer;
    this.conveyor = conveyor;
    this.drive = drive;
    this.speedBoostSupplier = speedBoostSupplier;
    this.sotmSupplier = sotmSupplier;
    this.multiplier = multiplier.getAsDouble();
    addRequirements(shooter, transfer, conveyor);
  }

  public HubSOTMShotCommand(Drive drive, Shooter shooter, Transfer transfer, Conveyor conveyor) {
    this(drive, shooter, transfer, conveyor, () -> false, () -> false, () -> 3.5);
  }

  public HubSOTMShotCommand(Drive drive, Shooter shooter, Transfer transfer, Conveyor conveyor,
      BooleanSupplier speedBoostSupplier) {
    this(drive, shooter, transfer, conveyor, speedBoostSupplier, () -> false, () -> 3.5);
  }

  public HubSOTMShotCommand(Drive drive, Shooter shooter, Transfer transfer, Conveyor conveyor,
      BooleanSupplier speedBoostSupplier, BooleanSupplier sotmSupplier) {
    this(drive, shooter, transfer, conveyor, speedBoostSupplier, sotmSupplier, () -> 3.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hopperRampTimer.reset();
    hopperRampTimer.start();
    transfer.feedTimer.reset();
    transfer.feeding = false;
    TIMER_DELAY = multiplier > 2.5 ? 0.0 : TIMER_DELAY;
    Logger.recordOutput(getName() + "/speedBoostSupplier", speedBoostSupplier.getAsBoolean());

    multiplier = 3.5 * (sotmSupplier.getAsBoolean() ? 0 : speedBoostSupplier.getAsBoolean() ? SPEED_BOOST_FACTOR : 1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean moving = drive.getChassisSpeeds().vxMetersPerSecond + drive.getChassisSpeeds().vyMetersPerSecond > 0.2;
    targetPose = Constants.FieldConstants.getAllianceHubPose();
    SOTMUtil.ShotSolution shot = SOTMUtil.computeConvergedShot(
        drive.getPose(),
        drive.getFieldRelativeChassisSpeeds(),
        targetPose,
        shooter::getVelo);
    distance = shot.distance();
    setpoint = shot.setpoint();
    if (moving) {
      setpoint = 30;
    }

    Logger.recordOutput(getName() + "/targetPose", targetPose);
    shooter.setBangBangSetpoint(setpoint - 0.3);

    Logger.recordOutput(getName() + "/distance", distance);
    Logger.recordOutput(getName() + "/setpoint", setpoint);

    shooter.setBangBangSetpoint(setpoint - 0.3);
    if (moving) {
      return;
    }

    if (Math.abs(shooter.getLeftShooterVelocity() - setpoint) < tolerance
        || Math.abs(shooter.getRightShooterVelocity() - setpoint) < tolerance || transfer.feeding) {
      transfer.feedTimer.start();
      transfer.feeding = true;
      if (drive.aligned) {
        transfer.setTransferDutyCycle(1);
        conveyor.setLeftFunnelDutyCycle(-1);

      } else {
        transfer.setTransferDutyCycle(0);
        conveyor.setLeftFunnelDutyCycle(0);
      }
    } else {
      transfer.feedTimer.stop();
      transfer.feedTimer.reset();
      transfer.setTransferDutyCycle(0);
    }
    TIMER_DELAY = speedBoostSupplier.getAsBoolean() ? 0 : 0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transfer.feeding = false;
    transfer.feedTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}