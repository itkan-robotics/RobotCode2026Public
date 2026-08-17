// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.IntakeConstants.INTAKE_PIVOT_DOWN;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DistanceBasedShotCommand extends Command {
  /** Creates a new RPMShootCommand. */
  // takes in the shooter the transfer conveyor intake and a rps
  // sets the shooter to the rps, once within threshold run transfer and coveyor
  // while running it bring the intake in halfway
  final Shooter shooter;
  final Intake intake;
  final Transfer transfer;
  final Conveyor conveyor;
  final Drive drive;
  private Timer feedTimer = new Timer();
  private boolean feeding = false;
  public double rps;
  public double distance;
  double setpoint;
  Pose2d targetPose;

  //Cap for agitation(how far inwards pivot goes)
  private double agitationCap = -35.0;
  private double TIMER_DELAY = 1;
  double multiplier = 1.6;

  private double agitatePos = INTAKE_PIVOT_DOWN;

  // tolerance in rps
  public double tolerance = 2;

  public DistanceBasedShotCommand(Drive drive, Shooter shooter, Intake intake, Transfer transfer, Conveyor conveyor, double multiplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.intake = intake;
    this.transfer = transfer;
    this.conveyor = conveyor;
    this.drive = drive;
    this.multiplier = multiplier;
    addRequirements(shooter, transfer, intake, conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feedTimer.reset();
    feeding = false;
    agitatePos = INTAKE_PIVOT_DOWN;
    intake.setIntakePosition(INTAKE_PIVOT_DOWN);
    TIMER_DELAY = multiplier >= 2.5 ? 0.4 : TIMER_DELAY;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetPose = Constants.FieldConstants.getAllianceHubPose();
    distance = drive.getPose().getTranslation().getDistance(targetPose.getTranslation());
    setpoint = shooter.getVelo(distance);
    // SmartDashboard.putNumber("DistanceBasedShot/distance", distance);
    // SmartDashboard.putNumber("DistanceBasedShot/setpoint", setpoint);
    shooter.setBangBangSetpoint(setpoint);

    if (Math.abs(shooter.getLeftShooterVelocity() - setpoint) < tolerance || feeding) {
      feedTimer.start();
      feeding = true;
      intake.setIntakeDutyCycle(0.7);
      transfer.setTransferDutyCycle(1);

      conveyor.setLeftFunnelDutyCycle(-0.8);
    }
    else{
      feedTimer.stop();
    }

    if (feeding) {
      agitatePos = Math.max(17.5, INTAKE_PIVOT_DOWN - (feedTimer.get() * 8.5));
    }
    intake.setIntakePosition(agitatePos);

    
     Logger.recordOutput(getName() + "/feeding", feeding);
     Logger.recordOutput(getName() + "/setpoint", setpoint);
    //  Logger.recordOutput(getName() + "/agitatePos", agitatePos);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}