package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionIO implements VisionIO {
  private final Drive drive;

  private final PhotonPoseEstimator photonEstimatorLeft;
  private final PhotonPoseEstimator photonEstimatorRight;
  private final PhotonCamera leftCamera;
  private final PhotonCamera rightCamera;

  private Matrix<N3, N1> curStdDevs;
  private PhotonPipelineResult result;

  // estConsumer: Lamba that will accept a pose estimate and pass it to your desired poseEstimator
  public PhotonVisionIO(Drive drive) {
    this.drive = drive;

    photonEstimatorLeft = new PhotonPoseEstimator(FIELD_TAG_LAYOUT, LEFT_ROBOT_TO_CAMERA);
    photonEstimatorRight = new PhotonPoseEstimator(FIELD_TAG_LAYOUT, RIGHT_ROBOT_TO_CAMERA);
    leftCamera = new PhotonCamera(LEFT_CAMERA_NAME);
    rightCamera = new PhotonCamera(RIGHT_CAMERA_NAME);
  }

  @Override
  public void processVision() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    for (PhotonPipelineResult cameraResult : leftCamera.getAllUnreadResults()) {
      result = cameraResult;
      visionEst = photonEstimatorLeft.estimateCoprocMultiTagPose(result);

      // fallback if multi-tag estimation fails
      if (visionEst.isEmpty()) {
        visionEst = photonEstimatorLeft.estimateLowestAmbiguityPose(result);
      }

      curStdDevs = updateEstimationStdDevs(visionEst, result.getTargets());

      // Lambda to handle Optional
      visionEst.ifPresent(
          est -> {
            drive.addVisionMeasurement(
                est.estimatedPose.toPose2d(), est.timestampSeconds, curStdDevs);
          });
    }

    for (PhotonPipelineResult cameraResult : rightCamera.getAllUnreadResults()) {
      result = cameraResult;
      visionEst = photonEstimatorRight.estimateCoprocMultiTagPose(result);

      // fallback if multi-tag estimation fails
      if (visionEst.isEmpty()) {
        visionEst = photonEstimatorRight.estimateLowestAmbiguityPose(result);
      }

      curStdDevs = updateEstimationStdDevs(visionEst, result.getTargets());

      // Lambda to handle Optional
      visionEst.ifPresent(
          est -> {
            drive.addVisionMeasurement(
                est.estimatedPose.toPose2d(), est.timestampSeconds, curStdDevs);
          });
    }
  }

  private Matrix<N3, N1> updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      return kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = photonEstimatorLeft.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        return kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
      }
    }
  }
}
