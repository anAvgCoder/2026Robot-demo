package frc.robot.subsystems.questNav;

import static frc.robot.subsystems.questNav.QuestNavConstants.ROBOT_TO_QUEST;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveConstants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class QuestNavSensor extends SubsystemBase {
  private final QuestNav quest;

  // quest pose data
  private PoseFrame[] latestPoseFrames;
  private ArrayList<PoseFrame> last6PoseFrames;
  private int preZeroFrameCount;

  private Pose3d defaultInitialPose;
  private Pose3d robotPose;

  // quest states
  private int batteryPercent;
  private boolean questWorking;
  private boolean questFlagged;
  private boolean flagConfirmed;
  private boolean ignoreFlags;
  private boolean poseJumpFlag;
  private boolean dofFlag;

  // saved velocities
  private double lastV;
  private double lastVx;
  private double lastVy;
  private double lastWz;

  public QuestNavSensor() {
    this.quest = new QuestNav();

    last6PoseFrames = new ArrayList<>();
    latestPoseFrames = new PoseFrame[] {};
    preZeroFrameCount = 0;

    defaultInitialPose = Pose3d.kZero;

    robotPose = defaultInitialPose;

    lastV = 0;
    lastVx = 0;
    lastVy = 0;
    lastWz = 0;

    zeroQuestPose(defaultInitialPose);

    ignoreFlags = false;
    clearFlags();
    Logger.recordOutput("QuestNav/Flags/ignoreFlags", false);
  }

  public void runPeriodicUpdates() {
    quest.commandPeriodic();

    readPoseFrames();
    // getLastFrameVelocity();
    hasFlags();
    isWorking();
  }

  public boolean isWorking() {
    questWorking = quest.isConnected() && quest.isTracking() && (!flagConfirmed || ignoreFlags);

    Logger.recordOutput("QuestSensor/isConnected", quest.isConnected());
    Logger.recordOutput("QuestSensor/isTracking", quest.isTracking());
    Logger.recordOutput("QuestSensor/isWorking", questWorking);
    return questWorking;
  }

  public void zeroQuestPose(Pose3d pose) {
    if (quest.isConnected()) {
      quest.setPose(pose);
      zeroPoseFrames(pose);
    }
  }

  // Must be called only ONE time per loop
  private void readPoseFrames() {
    if (quest.isTracking()) {
      latestPoseFrames = quest.getAllUnreadPoseFrames();
    }

    Pose3d questPose;

    for (PoseFrame frame : latestPoseFrames) {
      if (!frame.isTracking()) {
        continue;
      }

      Logger.recordOutput("QuestNav/PoseFrames", frame);
      samplePoseFrame(frame);

      questPose = frame.questPose3d();
      robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

      Logger.recordOutput("QuestNav/QuestPose", questPose);
      Logger.recordOutput("QuestNav/RobotPose", robotPose);
    }
  }

  private void samplePoseFrame(PoseFrame frame) {
    last6PoseFrames.add(frame);

    while (last6PoseFrames.size() > 6) {
      last6PoseFrames.remove(0);
    }
  }

  private void zeroPoseFrames(Pose3d pose) {
    last6PoseFrames.clear();

    if (latestPoseFrames.length > 0) {
      preZeroFrameCount = latestPoseFrames[latestPoseFrames.length - 1].frameCount();
    } else {
      preZeroFrameCount = 0;
    }

    PoseFrame zeroFrame =
        new PoseFrame(
            pose,
            RobotController.getFPGATime() / 1e6,
            quest.getAppTimestamp().getAsDouble(),
            preZeroFrameCount + 1,
            quest.isTracking());

    latestPoseFrames = new PoseFrame[] {zeroFrame};

    samplePoseFrame(zeroFrame);
  }

  public boolean hasFlags() {
    Logger.recordOutput("QuestNav/Flags/flagConfirmed", flagConfirmed);

    if (ignoreFlags) {
      clearFlags();
    } else if (flagConfirmed) {
      questFlagged = true;
    }

    if ((!flagConfirmed || ignoreFlags) && last6PoseFrames.size() > 1) {
      Pose3d lastPose = last6PoseFrames.get(last6PoseFrames.size() - 1).questPose3d();
      Pose3d firstPose = last6PoseFrames.get(0).questPose3d();

      dofFlag = lastPose.getX() == firstPose.getX() && lastPose.getY() == firstPose.getY();

      poseJumpFlag = lastV > DriveConstants.maxSpeedMetersPerSec;
      poseJumpFlag |= lastWz > DriveConstants.maxSpeedMetersPerSec / DriveConstants.driveBaseRadius;

      questFlagged = dofFlag || poseJumpFlag;
    }

    Logger.recordOutput("QuestNav/Flags/hasFlag", questFlagged);
    Logger.recordOutput("QuestNav/Flags/dofFlag", dofFlag);
    Logger.recordOutput("QuestNav/Flags/poseJumpFlag", poseJumpFlag);

    return questFlagged;
  }

  public void confirmFlag() {
    flagConfirmed = true;
  }

  public void clearFlags() {
    flagConfirmed = false;
    questFlagged = false;
    poseJumpFlag = false;
    dofFlag = false;

    Logger.recordOutput("QuestNav/Flags/flagsLastCleared", RobotController.getFPGATime() / 1e6);
  }

  public void toggleIgnoreFlags() {
    ignoreFlags = !ignoreFlags;
    Logger.recordOutput("QuestNav/ignoreFlags", ignoreFlags);

    if (ignoreFlags) {
      clearFlags();
    }
  }

  public Pose2d getRobotPose() {
    return robotPose.toPose2d();
  }

  public double getLinearVelocity() {
    return lastV;
  }

  public double getYawVelocity() {
    return lastWz;
  }

  public int getQuestBattery() {
    return batteryPercent;
  }
}