package frc.robot.util;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class Util {
  public static Color getHubColor() {
    String gameData;
    gameData = DriverStation.getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          // Blue case code
          return Color.kBlue;
        case 'R':
          // Red case code
          return Color.kRed;
        default:
          // This is corrupt data
          return Color.kMediumPurple;
      }
    } else {
      // Code for no data received yet
      return Color.kWhite;
    }
  }

  public static boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  static Field2d field;

  public static synchronized Field2d getUtilField() {
    if (field == null) {
      field = new Field2d();
      Logger.recordOutput("null", true);
    }
    Logger.recordOutput("null", false);
    // SmartDashboard.putData("automationRegions", field);
    return field;
  }

  public static void plotRectangles(Rectangle2d[] rects, String[] names) {
    for (int i = 0; i < rects.length; i++) {
      field.getObject(names[i]).setPoses(rectangleToPoses(rects[i]));
    }
  }

  private static List<Pose2d> rectangleToPoses(Rectangle2d rect) {
    Translation2d center = rect.getCenter().getTranslation();
    double x = rect.getXWidth() / 2.0;
    double y = rect.getYWidth() / 2.0;

    return List.of(
        new Pose2d(center.getX() - x, center.getY() - y, Rotation2d.kZero),
        new Pose2d(center.getX() - x, center.getY() + y, Rotation2d.kZero),
        new Pose2d(center.getX() + x, center.getY() + y, Rotation2d.kZero),
        new Pose2d(center.getX() + x, center.getY() - y, Rotation2d.kZero),
        new Pose2d(center.getX() - x, center.getY() - y, Rotation2d.kZero) // close loop
    );
  }

  @AutoLogOutput(key="LEDs/withinAAZ")
  public static boolean withinAutoAlignZone(Pose2d botPose) {
    // AutoAlign Code
      Rectangle2d autoAlignZone = Constants.isBlueAlliance()
          ? new Rectangle2d(new Translation2d(0, 8), new Translation2d(4, 0))
          : new Rectangle2d(new Translation2d(16.5, 8), new Translation2d(12.5, 0));

      boolean withinAutoAlignZone = autoAlignZone.contains(botPose.getTranslation());
      try {
        Logger.recordOutput("LEDs/withinAAZ", withinAutoAlignZone);
        return withinAutoAlignZone;
      } catch (Exception e) {
        return false;
      }
  }
}
