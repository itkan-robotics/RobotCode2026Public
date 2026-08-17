// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics
 * sim) and "replay"
 * (log replay from a file).
 *
 * Robot-specific constants are pinned to the team 9128 values.
 * Edit values here when retuning.
 */

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final boolean tuningMode = true;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class PathPlannerConstants {
    public static final double ROBOT_MASS_KG = 74;
    public static final double ROBOT_MOI = 9.75;
    public static double WHEEL_COF = 1.2;
    public static LoggedTunableNumber tunableTranslateAutoP = new LoggedTunableNumber("tunableTranslateAutoP", 2.50);

    public static double translationalAutoP = tunableTranslateAutoP.getAsDouble();

    public static LoggedTunableNumber tunableRotationalAutoP = new LoggedTunableNumber("tunableRotationalAutoP", 3.25);

    public static double rotationalAutoP = tunableRotationalAutoP.getAsDouble();
  }

  public static class FieldConstants {
    public static final double GOAL_HEIGHT = 0;
    public static final Pose2d BLUE_GOAL_POSE = new Pose2d(4.630, 4.050, Rotation2d.kZero);
    public static final Pose2d RED_GOAL_POSE = new Pose2d(11.920, 4.050, Rotation2d.kZero);

    public static final Pose2d TOP_RED_PASSING_POSE = new Pose2d(15, 6, Rotation2d.kZero);
    public static final Pose2d BOTTOM_RED_PASSING_POSE = new Pose2d(15, 2, Rotation2d.kZero);
    public static final Pose2d TOP_BLUE_PASSING_POSE = new Pose2d(2, 6, Rotation2d.kZero);
    public static final Pose2d BOTTOM_BLUE_PASSING_POSE = new Pose2d(2, 2, Rotation2d.kZero);
    public static final Pose2d[] BLUE_PASSING_POSES = new Pose2d[] { TOP_BLUE_PASSING_POSE, BOTTOM_BLUE_PASSING_POSE };
    public static final Pose2d[] RED_PASSING_POSES = new Pose2d[] { TOP_RED_PASSING_POSE, BOTTOM_RED_PASSING_POSE };
    static double kHubOffsetMeters = 0.275;
    public static final Rectangle2d BLUE_HUB_SWEEEP = new Rectangle2d(
        new Translation2d(5.15+kHubOffsetMeters, 0.0),
        new Translation2d(6.25+kHubOffsetMeters, 8.0));
    public static final Rectangle2d RED_HUB_SWEEP = new Rectangle2d(
        new Translation2d(11.25-kHubOffsetMeters, 0.0),
        new Translation2d(10.25-kHubOffsetMeters, 8.0));
        static double kTowerOffsetMeters = 0.10;
    public static final Rectangle2d RED_TOWER_SWEEP = new Rectangle2d(
        new Translation2d(15.5+kTowerOffsetMeters, 2.0),
        new Translation2d(16.5+kTowerOffsetMeters, 6.0));
    public static final Rectangle2d BLUE_TOWER_SWEEP = new Rectangle2d(
        new Translation2d(1.0-kTowerOffsetMeters, 2.0),
        new Translation2d(0-kTowerOffsetMeters, 6.0));
    public static Rectangle2d[] sweepRegions = new Rectangle2d[] { BLUE_HUB_SWEEEP, RED_HUB_SWEEP, BLUE_TOWER_SWEEP,
        RED_TOWER_SWEEP };
    public static String[] sweepNames = new String[] { "blueHub", "redHub", "blueTower", "redTower" };

    public static final Rectangle2d[] neutralZoneRectangles = new Rectangle2d[] {
        new Rectangle2d(new Translation2d(4.5, 8.0), new Translation2d(12.0, 4.0)),
        new Rectangle2d(new Translation2d(4.5, 0.0), new Translation2d(12.0, 4.0))
    };
    public static final String[] neutralZoneNames = new String[] { "closeNeutralZone", "farNeutralZone" };

    /** If blue alliance, get blue goal pose; else, get red goal pose */
    public static Pose2d getAllianceHubPose() {
      return Constants.isBlueAlliance()
          ? BLUE_GOAL_POSE
          : RED_GOAL_POSE;
    }

    public static Pose2d getPassingPose(Pose2d currPose) {
      Pose2d[] possiblePassingPoses = Constants.isBlueAlliance() ? BLUE_PASSING_POSES : RED_PASSING_POSES;
      Util.getUtilField();
      Util.plotRectangles(neutralZoneRectangles, neutralZoneNames);
      return currPose.getY() > 4.0 ? possiblePassingPoses[0] : possiblePassingPoses[1];
    }

    public static Pose2d getSweepingPose(Pose2d currPose) {
      Util.getUtilField();
      Util.plotRectangles(sweepRegions, sweepNames);
      for (Rectangle2d sweepRegion : sweepRegions) {
        if (sweepRegion.contains(currPose.getTranslation()))
          return new Pose2d(sweepRegion.getCenter().getTranslation(), getRotationFromCenter(sweepRegion.getCenter()));
      }
      return currPose;
    }

    public static Rotation2d getRotationFromCenter(Pose2d centerPose) {
      if (centerPose == BLUE_HUB_SWEEEP.getCenter())
        return new Rotation2d(Math.toRadians(135));
      if (centerPose == RED_HUB_SWEEP.getCenter())
        return new Rotation2d(Math.toRadians(45));
      if (centerPose == RED_TOWER_SWEEP.getCenter() || centerPose == BLUE_TOWER_SWEEP.getCenter())
        return new Rotation2d(Math.toRadians(90.0));

      return Rotation2d.kZero;
    }
  }

  public static boolean isBlueAlliance() {
    try {
      return DriverStation.getAlliance().get().equals(Alliance.Blue);
    } catch (Exception e) {
      // TODO: handle exception
      return false;
    }
  }

  public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "limelight-front";

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = Double.POSITIVE_INFINITY; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
    public static final double LIMELIGHT_ANGLE = 0;
    public static final double LIMELIGHT_HEIGHT = 0;
  }

  public static class ShooterConstants {
    public static final int LEFT_SHOOTER_PORT = 16;
    public static final int RIGHT_SHOOTER_PORT = 13;
    public static final double SHOOTER_KS = 0.16;
    public static final double SHOOTER_KV = 0.1199;
    public static final double SHOOTER_KA = 0;
    public static final double SHOOTER_KG = 0;
    public static final double SHOOTER_KP = 0.4;
    public static final double SHOOTER_KI = 0;
    public static final double SHOOTER_KD = 0;
    public static final double SHOOTER_FF = 0;
    public static final double SHOOTER_TOLERANCE = 0;
  }

  public static class IntakeConstants {
    public static final int FRONT_INTAKE_RIGHT_MOTOR_PORT = 23;
    public static final int FRONT_INTAKE_LEFT_MOTOR_PORT = 22;
    public static final int INTAKE_PIVOT_MOTOR_PORT = 17;
    public static final double INTAKE_PIVOT_KS = 0.0;
    public static final double INTAKE_PIVOT_KV = 0;
    public static final double INTAKE_PIVOT_KA = 0;
    public static final double INTAKE_PIVOT_KG = 0;
    public static final double INTAKE_PIVOT_KP = 12.0;
    public static final double INTAKE_PIVOT_KI = 0;
    public static final double INTAKE_PIVOT_KD = 0.0000;
    public static final double INTAKE_PIVOT_VELOCITY = 125;
    public static final double INTAKE_PIVOT_ACCELERATION = 375;
    public static final double INTAKE_PIVOT_JERK = 1675;
    public static final double INTAKE_PIVOT_FF = 0;
    public static final double INTAKE_PIVOT_TOLERANCE = 1;
    public static double INTAKE_PIVOT_DOWN = 41.5;
    public static double INTAKE_AGITATE_DOWN = 40;
    public static final double INTAKE_PIVOT_UP = 0;
    public static final double INTAKE_HOME_POSITION = -45;
  }

  public static class ConveyorConstants {
    public static final int CONVEYOR_MOTOR_PORT = 15;
  }

  public static class TransferConstants {
    public static final int RIGHT_TRANSFER_MOTOR_PORT = 19;
    public static final int LEFT_TRANSFER_MOTOR_PORT = 18;
    public static final int TRANSFER_IDLE_STATOR_CURRENT = 10;
  }

}
