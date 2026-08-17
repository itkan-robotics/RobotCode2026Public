package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleUnaryOperator;

public final class SOTMUtil {
  private SOTMUtil() {
  }

  public record ShotSolution(double distance, double setpoint) {
  }

  public static double timeOfFlight(double distanceMeters) {
    return .0220691 * distanceMeters * distanceMeters + .0691981 * distanceMeters + 0.7803;
  }

  public static Pose2d getVirtualGoalPose(
      double timeSeconds,
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeChassisSpeeds,
      Pose2d originalTargetPose) {
    return new Pose2d(
        new Translation2d(
            originalTargetPose.getX() - timeSeconds * fieldRelativeChassisSpeeds.vxMetersPerSecond,
            originalTargetPose.getY() - timeSeconds * fieldRelativeChassisSpeeds.vyMetersPerSecond),
        new Rotation2d());
  }

  public static double getVirtualGoalDistance(
      double timeSeconds,
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeChassisSpeeds,
      Pose2d originalTargetPose) {
    return robotPose.getTranslation()
        .getDistance(getVirtualGoalPose(timeSeconds, robotPose, fieldRelativeChassisSpeeds, originalTargetPose).getTranslation());
  }

  /** Iteratively converges distance and shooter setpoint for a moving robot. */
  public static ShotSolution computeConvergedShot(
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeChassisSpeeds,
      Pose2d targetPose,
      DoubleUnaryOperator velocityFromDistance) {
    double distance = robotPose.getTranslation().getDistance(targetPose.getTranslation());
    double setpoint = velocityFromDistance.applyAsDouble(distance);
    double lastSetpoint = 0;
    while (Math.abs(setpoint - lastSetpoint) > 0.1) {
      double time = timeOfFlight(distance);
      distance = getVirtualGoalDistance(
          time, robotPose, fieldRelativeChassisSpeeds, targetPose);
      lastSetpoint = setpoint;
      setpoint = velocityFromDistance.applyAsDouble(distance);
    }
    return new ShotSolution(distance, setpoint);
  }
}
