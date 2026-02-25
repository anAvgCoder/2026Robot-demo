package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final Pose3d BLUE_LOW_TARGET_POSE3D =
      new Pose3d(
          Units.inchesToMeters(42),
          Units.inchesToMeters(317.69 * 1 / 4),
          Units.inchesToMeters(0.0),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));

  public static final Pose3d BLUE_HIGH_TARGET_POSE3D =
      new Pose3d(
          Units.inchesToMeters(42),
          Units.inchesToMeters(317.69 * 3 / 4),
          Units.inchesToMeters(0.0),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));

  public static final Pose3d BLUE_HUB_POSE3D =
      new Pose3d(
          Units.inchesToMeters(182.11),
          Units.inchesToMeters(317.69 / 2),
          Units.inchesToMeters(0.0),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));

  public static final Pose3d RED_LOW_TARGET_POSE3D =
      new Pose3d(
          Units.inchesToMeters(651.22 - 42),
          Units.inchesToMeters(317.69 * 1 / 4),
          Units.inchesToMeters(0.0),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));

  public static final Pose3d RED_HIGH_TARGET_POSE3D =
      new Pose3d(
          Units.inchesToMeters(651.22 - 42),
          Units.inchesToMeters(317.69 * 3 / 4),
          Units.inchesToMeters(0.0),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));

  public static final Pose3d RED_HUB_POSE3D =
      new Pose3d(
          Units.inchesToMeters(651.22 - 182.11),
          Units.inchesToMeters(317.69 / 2),
          Units.inchesToMeters(0.0),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));
}
