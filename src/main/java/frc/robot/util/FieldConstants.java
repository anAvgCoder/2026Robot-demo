package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {

  public static final double UPPIE_DOWNIE_INCHES = 10.0;

  public static final double FIELD_WIDTH_INCHES = 317.69;
  public static final double FIELD_LENGTH_INCHES = 651.22;
  public static final double FIELD_HUB_TO_SIDE_INCHES = 182.11;

  // STORAGE TARGETS TO AIM FOR
  public static final Pose3d BLUE_LOW_TARGET_POSE3D =
      new Pose3d(
          Units.inchesToMeters(42),
          Units.inchesToMeters(FIELD_LENGTH_INCHES * 1 / 4),
          Units.inchesToMeters(0.0),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));
  public static final Pose3d BLUE_HIGH_TARGET_POSE3D =
      new Pose3d(
          Units.inchesToMeters(42),
          Units.inchesToMeters(FIELD_LENGTH_INCHES * 3 / 4),
          Units.inchesToMeters(0.0),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));

  public static final Pose3d RED_LOW_TARGET_POSE3D =
      new Pose3d(
          Units.inchesToMeters(FIELD_WIDTH_INCHES - 42),
          Units.inchesToMeters(FIELD_LENGTH_INCHES * 1 / 4),
          Units.inchesToMeters(0.0),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));

  public static final Pose3d RED_HIGH_TARGET_POSE3D =
      new Pose3d(
          Units.inchesToMeters(FIELD_WIDTH_INCHES - 42),
          Units.inchesToMeters(FIELD_LENGTH_INCHES * 3 / 4),
          Units.inchesToMeters(0.0),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));

  // HUB POSES
  public static final Pose3d BLUE_HUB_POSE3D =
      new Pose3d(
          Units.inchesToMeters(FIELD_HUB_TO_SIDE_INCHES),
          Units.inchesToMeters(FIELD_WIDTH_INCHES / 2),
          Units.inchesToMeters(0.0),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));

  public static final Pose3d RED_HUB_POSE3D =
      new Pose3d(
          Units.inchesToMeters(FIELD_LENGTH_INCHES - FIELD_HUB_TO_SIDE_INCHES),
          Units.inchesToMeters(FIELD_WIDTH_INCHES / 2),
          Units.inchesToMeters(0.0),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));

  public static final Pose3d BLUE_OUTPOST_POSE3D =
      new Pose3d(
          Units.inchesToMeters(24), // x field length
          Units.inchesToMeters(60), // y field width
          Units.inchesToMeters(0.0),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));

  public static final Pose3d RED_OUTPOST_POSE3D =
      new Pose3d(
          Units.inchesToMeters(FIELD_LENGTH_INCHES - 24),
          Units.inchesToMeters(FIELD_WIDTH_INCHES - 60),
          Units.inchesToMeters(0.0),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));

  public static final Pose3d BLUE_DEPOT_POSE3D =
      new Pose3d(
          Units.inchesToMeters(36), // x field length
          Units.inchesToMeters(FIELD_WIDTH_INCHES - 60), // y field width
          Units.inchesToMeters(0.0),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));

  public static final Pose3d RED_DEPOT_POSE3D =
      new Pose3d(
          Units.inchesToMeters(FIELD_LENGTH_INCHES - 36),
          Units.inchesToMeters(60),
          Units.inchesToMeters(0.0),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));
}
