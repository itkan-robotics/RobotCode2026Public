// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RPMShootCommand extends Command {
  /** Creates a new RPMShootCommand. */
  // takes in the shooter the transfer conveyor intake and a rps
  // sets the shooter to the rps, once within threshold run transfer and coveyor
  // while running it bring the intake in halfway
  final Shooter shooter;
  final Transfer transfer;
  final Conveyor conveyor;
  public double rps;
  double tolerance = 2;

  public RPMShootCommand(Shooter shooter, Transfer transfer, Conveyor conveyor, DoubleSupplier rps) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.transfer = transfer;
    this.conveyor = conveyor;
    this.rps = rps.getAsDouble();
    addRequirements(shooter, transfer, conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transfer.feedTimer.reset();
    transfer.feeding = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooter.setBangBangSetpoint(rps);

    if (Math.abs(shooter.getLeftShooterVelocity() - rps) < tolerance
        || Math.abs(shooter.getRightShooterVelocity() - rps) < tolerance || transfer.feeding) {
      transfer.feedTimer.start();
      transfer.feeding = true;
      transfer.setTransferDutyCycle(1);
      conveyor.setLeftFunnelDutyCycle(-1);
    }
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
