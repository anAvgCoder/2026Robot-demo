package frc.robot.subsystems.questnav;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.util.FieldConstants;

public class QuestNavConstants {

  public static double ROBOT_TO_QUEST_INCHES_X_DOUBLE = 10.75;
  public static double ROBOT_TO_QUEST_INCHES_Z_DOUBLE = 13.5;

  public static double ROBOT_WIDTH_INCHES_BUMPER_HALF_DOUBLE =
      17.125; // 27/2 + 3.25 (bumpers) + 6/16 (brackets)
  public static double ROBOT_LENGTH_INCHES_BUMPER__HALF_DOUBLE =
      17; // 27/2 + 3.125 (bumpers) + 6/16 (brackets)

  // Calculation inputs
  public static final Transform3d ROBOT_TO_QUEST =
      new Transform3d(
          Units.inchesToMeters(ROBOT_TO_QUEST_INCHES_X_DOUBLE),
          0,
          Units.inchesToMeters(ROBOT_TO_QUEST_INCHES_Z_DOUBLE),
          new Rotation3d(0, 0, 0));

  public static final Pose3d ROBOT_TO_QUEST_RED =
      new Pose3d(
          Units.inchesToMeters(
              FieldConstants.FIELD_LENGTH_INCHES
                  - ROBOT_WIDTH_INCHES_BUMPER_HALF_DOUBLE
                  - ROBOT_TO_QUEST_INCHES_X_DOUBLE),
          Units.inchesToMeters(ROBOT_LENGTH_INCHES_BUMPER__HALF_DOUBLE),
          Units.inchesToMeters(ROBOT_TO_QUEST_INCHES_Z_DOUBLE),
          new Rotation3d(0, 0, Units.degreesToRadians(180)));

  public static final Pose3d ROBOT_TO_QUEST_BLUE =
      new Pose3d(
          Units.inchesToMeters(
              ROBOT_WIDTH_INCHES_BUMPER_HALF_DOUBLE + ROBOT_TO_QUEST_INCHES_X_DOUBLE),
          Units.inchesToMeters(
              FieldConstants.FIELD_WIDTH_INCHES - ROBOT_LENGTH_INCHES_BUMPER__HALF_DOUBLE),
          Units.inchesToMeters(ROBOT_TO_QUEST_INCHES_Z_DOUBLE),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));

  public static final Pose3d ROBOT_TO_QUEST_BLUE_HUB =
      new Pose3d(
          Units.inchesToMeters(
              156.61 - ROBOT_WIDTH_INCHES_BUMPER_HALF_DOUBLE + ROBOT_TO_QUEST_INCHES_X_DOUBLE),
          Units.inchesToMeters(FieldConstants.FIELD_WIDTH_INCHES / 2),
          Units.inchesToMeters(ROBOT_TO_QUEST_INCHES_Z_DOUBLE),
          new Rotation3d(0, 0, Units.degreesToRadians(0)));

  public static final Pose3d ROBOT_TO_QUEST_RED_HUB =
      new Pose3d(
          Units.inchesToMeters(
              651.22
                  - 182.11
                  + 47.00 / 2
                  + ROBOT_WIDTH_INCHES_BUMPER_HALF_DOUBLE
                  - ROBOT_TO_QUEST_INCHES_X_DOUBLE),
          Units.inchesToMeters(FieldConstants.FIELD_WIDTH_INCHES / 2),
          Units.inchesToMeters(ROBOT_TO_QUEST_INCHES_Z_DOUBLE),
          new Rotation3d(0, 0, Units.degreesToRadians(180)));

  public static final Matrix<N3, N1> questNavStdDevs =
      VecBuilder.fill(
          0.02, // Trust down to 2cm in X direction
          0.02, // Trust down to 2cm in Y direction
          0.035 // Trust down to 2 degrees rotational
          );

  public static final boolean overwritePoseEstimator = false;
}
