package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.FIELD_TAG_LAYOUT;
import static frc.robot.subsystems.vision.VisionConstants.LEFT_CAMERA_NAME;
import static frc.robot.subsystems.vision.VisionConstants.LEFT_ROBOT_TO_CAMERA;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class PhotonVisionSimIO implements VisionIO {
  private PhotonCameraSim leftCameraSim;
  // private PhotonCameraSim rightCameraSim;
  private VisionSystemSim visionSim;

  public PhotonVisionSimIO() {
    PhotonCamera leftCamera = new PhotonCamera(LEFT_CAMERA_NAME);
    // PhotonCamera rightCamera = new PhotonCamera(RIGHT_CAMERA_NAME);

    if (Robot.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(FIELD_TAG_LAYOUT);
      // Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(320, 240, Rotation2d.fromDegrees(90));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(70);
      cameraProp.setAvgLatencyMs(30);
      cameraProp.setLatencyStdDevMs(10);
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
      // targets.
      leftCameraSim = new PhotonCameraSim(leftCamera, cameraProp);
      // rightCameraSim = new PhotonCameraSim(rightCamera, cameraProp);
      // Add the simulated camera to view the targets on this simulated field.
      visionSim.addCamera(leftCameraSim, LEFT_ROBOT_TO_CAMERA);
      // visionSim.addCamera(rightCameraSim, RIGHT_ROBOT_TO_CAMERA);

      leftCameraSim.enableDrawWireframe(true);
      // rightCameraSim.enableDrawWireframe(true);
    }
  }

  @Override
  public void processVision() {
    visionSim.update(visionSim.getRobotPose());
  }
}
