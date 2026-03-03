package frc.robot.subsystems.questnav;

import static frc.robot.subsystems.questnav.QuestNavSystemConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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

  private final Pose3d[] last6 = new Pose3d[6];
  private int last6Idx = 0;
  private int last6Count = 0;

  public QuestNavSystemIOReal() {
    this.questNav = new QuestNav();

    questWorking = isWorking();

    // set initial reference point
    this.questNav.setPose(QuestNavSystemConstants.ROBOT_TO_QUEST_BLUE);
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
  public void resetQuestPoseZero(Pose3d pose3d) {
    this.questNav.setPose(pose3d);
  }

  // Test fix: this command may overwrite the value with nothing if called twice
  // per loop?
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

    if (questWorking && latestPoseFrames.length != 0) {
      setQuestPose(
          latestPoseFrames[latestPoseFrames.length - 1]
              .questPose3d()
              .transformBy(QuestNavSystemConstants.ROBOT_TO_QUEST.inverse()));
    }

    pushPose(robotPose);
    return robotPose;
  }

  private void pushPose(Pose3d pose) {
    last6[last6Idx] = pose;
    last6Idx = (last6Idx + 1) % last6.length;
    if (last6Count < last6.length) last6Count++;
  }

  @Override
  public Pose3d[] getLast6RobotPoses() {
    Pose3d[] out = new Pose3d[last6Count];
    int start = (last6Idx - last6Count + last6.length) % last6.length;
    for (int i = 0; i < last6Count; i++) {
      out[i] = last6[(start + i) % last6.length];
    }
    return out;
  }

  @Override
  public Pose3d predictPoseFromWindow(Pose3d[] poses, double tSeconds) {
    if (poses == null || poses.length < 2) {
      return new Pose3d();
    }

    Pose3d first = poses[0];
    Pose3d last = poses[poses.length - 1];

    final double dtWindow = (poses.length - 1) * 0.02;

    double vx = (last.getX() - first.getX()) / dtWindow; // m/s
    double vy = (last.getY() - first.getY()) / dtWindow; // m/s

    Rotation3d r0 = first.getRotation();
    Rotation3d r1 = last.getRotation();

    double dRoll = MathUtil.angleModulus(r1.getX() - r0.getX());
    double dPitch = MathUtil.angleModulus(r1.getY() - r0.getY());
    double dYaw = MathUtil.angleModulus(r1.getZ() - r0.getZ());

    double wx = dRoll / dtWindow;
    double wy = dPitch / dtWindow;
    double wz = dYaw / dtWindow;

    Rotation3d rPred =
        new Rotation3d(
            r1.getX() + wx * tSeconds, r1.getY() + wy * tSeconds, r1.getZ() + wz * tSeconds);

    return new Pose3d(last.getX() + vx * tSeconds, last.getY() + vy * tSeconds, last.getZ(), rPred);
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
