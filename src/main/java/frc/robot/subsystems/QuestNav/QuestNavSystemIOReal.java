package frc.robot.subsystems.questnav;

import static frc.robot.subsystems.questnav.QuestNavSystemConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;

public class QuestNavSystemIOReal implements QuestNavSystemIO {
  private final QuestNav questNav;

  // quest states (updated periodically)
  private boolean questWorking;
  private double batteryPercent;
  private PoseFrame[] latestPoseFrames;

  // stored poses
  private Pose3d robotPose;
  private Pose3d questPose;

  public QuestNavSystemIOReal() {
    this.questNav = new QuestNav();

    questWorking = isWorking();

    // set initial reference point
    setQuestPose(defaultQuestPose);
  }

  @Override
  public boolean isWorking() {
    questWorking = questNav.isConnected() && questNav.isTracking();

    return questWorking;
  }

  @Override
  public void setQuestPose(Pose3d pose3d) {
    questPose = pose3d;
    robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
  }

  @Override
  public void setRobotPose(Pose3d pose3d) {
    robotPose = pose3d;
    questPose = robotPose.transformBy(ROBOT_TO_QUEST);
  }

  @Override
  public void resetQuestPose(Pose3d pose3d) {
    setQuestPose(defaultQuestPose);
  }

  @Override
  public void resetQuestPoseZero(Pose3d pose3d) {
    defaultQuestPose = pose3d;
  }

  // Test fix: this command may overwrite the value with nothing if called twice per loop?
  @Override
  public void updateLatestPoseFrames() {
    latestPoseFrames = questNav.getAllUnreadPoseFrames();
  }

  @Override
  public PoseFrame[] getLatestPoseFrames() {
    return latestPoseFrames;
  }

  @Override
  public double getQuestVelocity() {
    // Implementation for getting quest velocity
    return 0;
  }

  @Override
  public Pose3d getLastRobotPose() {
    Logger.recordOutput("QuestNavTest/questWorking", questWorking);
    Logger.recordOutput("QuestNavTest/latestPoseFrames", latestPoseFrames);

    Transform3d transform = defaultQuestPose.minus(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));

    if (questWorking && latestPoseFrames.length != 0) {
      setQuestPose(
          latestPoseFrames[latestPoseFrames.length - 1].questPose3d().transformBy(transform));
    }

    return robotPose;
  }

  @Override
  public double getQuestBattery() {
    if (questWorking) {
      batteryPercent = questNav.getBatteryPercent().getAsInt();
    } else {
      batteryPercent = 0;
    }

    return batteryPercent;
  }

  @Override
  public void updateQuestStatus() {
    isWorking();
    getQuestBattery();
    updateLatestPoseFrames();
  }

  @Override
  public void runQuestCommand() {
    questNav.commandPeriodic();
  }
}
