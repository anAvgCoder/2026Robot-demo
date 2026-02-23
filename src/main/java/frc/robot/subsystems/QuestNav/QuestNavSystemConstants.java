package frc.robot.subsystems.questnav;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class QuestNavSystemConstants {

  // Calculation inputs
  public static final Transform3d ROBOT_TO_QUEST =
      new Transform3d(
          Units.inchesToMeters(-10.75), 0, Units.inchesToMeters(-13.5), new Rotation3d(0, 0, 0));

  // FIELD IS 651.22 INCHES WIDE and 317.69 INCHES LONG
  public static final Pose3d ROBOT_TO_QUEST_RED =
      new Pose3d(
              Units.inchesToMeters(651.22 - 17.125),
              Units.inchesToMeters(317.69 - 17),
              Units.inchesToMeters(-13.5),
              new Rotation3d(0, 0, Units.degreesToRadians(-90)))
          .transformBy(ROBOT_TO_QUEST);

  public static final Pose3d ROBOT_TO_QUEST_BLUE =
      new Pose3d(
              Units.inchesToMeters(17.125), // 27/2 + 3.25 (bumpers) + 6/16 (brackets)
              Units.inchesToMeters(17), // 27/2 + 3.125 (bumpers) + 6/16 (brackets)
              Units.inchesToMeters(-13.5),
              new Rotation3d(0, 0, Units.degreesToRadians(90)))
          .transformBy(ROBOT_TO_QUEST);

  public static final Matrix<N3, N1> questNavStdDevs =
      VecBuilder.fill(
          0.02, // Trust down to 2cm in X direction
          0.02, // Trust down to 2cm in Y direction
          0.035 // Trust down to 2 degrees rotational
          );

  public static Pose3d defaultQuestPose = Pose3d.kZero.transformBy(ROBOT_TO_QUEST);

  //
  public static final boolean overwritePoseEstimator = false;
}
