package frc.robot.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Overrides PathPlanner's rotation feedback with SOTM (shoot-on-the-move)
 */

public class SotmHeadingOverrideCommand extends Command {
    private final Drive drive;
    private final Shooter shooter;
    private final PIDController pid;
    private static final double ANGLE_KP = 0.12;

    public SotmHeadingOverrideCommand(Drive drive, Shooter shooter) {
        this.drive = drive;
        this.shooter = shooter;
        this.pid = new PIDController(ANGLE_KP * 0.9, 0, 0.001);
        this.pid.enableContinuousInput(-180, 180);
    }

    @Override
    public void initialize() {
        pid.reset();
        Pose2d robotDisplacementFromTarget = DriveCommands.getSotmDisplacementFromTarget(drive, shooter, false);
        PPHolonomicDriveController.overrideRotationFeedback(
                () -> DriveCommands.calculateOmegaFromDriverOrTarget(
          drive,
          ()->0,
          ()->0,
          () -> robotDisplacementFromTarget.getY(),
          () -> robotDisplacementFromTarget.getX(),
          DriveCommands.isAllianceFlipped()));
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        PPHolonomicDriveController.clearRotationFeedbackOverride();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
