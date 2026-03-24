package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  public static final String LEFT_CAMERA_NAME = "frontLeftPhotonVision";
  public static final String RIGHT_CAMERA_NAME = "frontRightPhotonVision";

  private static final double LEFT_CAMERA_PITCH = Units.degreesToRadians(22);
  private static final double LEFT_CAMERA_YAW = Units.degreesToRadians(45); // 35.5

  private static final double RIGHT_CAMERA_PITCH = Units.degreesToRadians(22.0);
  private static final double RIGHT_CAMERA_YAW = Units.degreesToRadians(-41.0);

  public static final Transform3d LEFT_ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(13), Units.inchesToMeters(12.5), Units.inchesToMeters(13.5)),
          new Rotation3d(0, LEFT_CAMERA_PITCH, LEFT_CAMERA_YAW));
  public static final Transform3d RIGHT_ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(13), Units.inchesToMeters(-12.5), Units.inchesToMeters(13.5)),
          new Rotation3d(0, RIGHT_CAMERA_PITCH, RIGHT_CAMERA_YAW));

  // The layout of the AprilTags on the field
  public static final AprilTagFieldLayout FIELD_TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
}
