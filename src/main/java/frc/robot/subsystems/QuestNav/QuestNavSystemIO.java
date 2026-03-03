package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose3d;
import gg.questnav.questnav.PoseFrame;

public interface QuestNavSystemIO {

  public default void updateQuestStatus() {}

  public default boolean isWorking() {
    return false;
  }

  public default void setQuestPose(Pose3d pose3d) {}

  public default void setRobotPose(Pose3d pose3d) {}

  public default double getQuestVelocity() {
    return 0;
  }

  public default void getQuestVector(Pose3d pose3d) {}

  public default void resetQuestPoseZero(Pose3d pose3d) {}

  public default void updateLatestPoseFrames() {}

  public default PoseFrame[] getLatestPoseFrames() {
    return new PoseFrame[0];
  }

  public default Pose3d getLastRobotPose() {
    return Pose3d.kZero;
  }

  public default Pose3d[] getLast6RobotPoses() {
    return new Pose3d[0];
  }

  public default Pose3d predictPoseFromWindow(Pose3d[] poses, double tSeconds) {
    return new Pose3d();
  }

  public default double getQuestBattery() {
    return 0;
  }

  public default void runQuestCommand() {}
}
