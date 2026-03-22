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
      if (frame.isTracking()) {
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

    // Remove old poseFrames
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

      // check only rotation updates
      dofFlag = lastPose.getX() == firstPose.getX() && lastPose.getY() == firstPose.getY();

      // check pose jumps
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

  // Uses equations of motion to predict robot x, y, and yaw from last 6 robot poses
  // z, roll & pitch are copied from last pose sample
  public Pose3d predictQuestPoseFromFrames(double targetTimestamp, boolean step2) {
    int size = last6PoseFrames.size();
    // distance between pose indexes
    int step = 1;

    if (step2 && size > 4) {
      step++;
    }

    double tSeconds = targetTimestamp - RobotController.getFPGATime() / 1e6;

    // Pose frames
    PoseFrame last = last6PoseFrames.get(size - 1);
    PoseFrame finalRef = last6PoseFrames.get(size - step - 1);

    PoseFrame first = last6PoseFrames.get(0);
    PoseFrame initRef = last6PoseFrames.get(step);

    // Poses
    Pose3d firstPose = first.questPose3d();
    Pose3d initRefPose = initRef.questPose3d();

    Pose3d lastPose = last.questPose3d();
    Pose3d finalRefPose = finalRef.questPose3d();

    Rotation3d lastRot = lastPose.getRotation();

    // Timestamps
    double firstTime = first.dataTimestamp();
    double initRefTime = initRef.dataTimestamp();

    double lastTime = last.dataTimestamp();
    double finalRefTime = finalRef.dataTimestamp();

    // seconds
    double dt = lastTime - firstTime;

    double ax = 0;
    double ay = 0;
    double aYaw = 0;

    double t1 = (initRefTime - firstTime);
    double t2 = (lastTime - finalRefTime);

    // meters / second
    double vix = (initRefPose.getX() - firstPose.getX()) / t1;
    double viy = (initRefPose.getY() - firstPose.getY()) / t1;

    double vfx = (lastPose.getX() - finalRefPose.getX()) / t2;
    double vfy = (lastPose.getY() - finalRefPose.getY()) / t2;

    // radians / second
    double wiz = (initRefPose.getRotation().getZ() - firstPose.getRotation().getZ()) / t2;
    double wfz = (lastPose.getRotation().getZ() - finalRefPose.getRotation().getZ()) / t2;

    if (t1 + t2 != dt) {
      ax = (vfx - vix) / (dt - (t1 + t2));
      ay = (vfy - viy) / (dt - (t1 + t2));
      aYaw = (wfz - wiz) / (dt - (t1 + t2));
    }

    double xPred = lastPose.getX() + vfx * tSeconds + ax * tSeconds * tSeconds;
    double yPred = lastPose.getY() + vfy * tSeconds + ay * tSeconds * tSeconds;

    Rotation3d rotPred =
        new Rotation3d(
            lastRot.getX(),
            lastRot.getY(),
            lastRot.getZ() + wfz * tSeconds + aYaw * tSeconds * tSeconds);

    return new Pose3d(xPred, yPred, lastPose.getZ(), rotPred);
  }

  public Pose2d getRobotPose() {
    return robotPose.toPose2d();
  }

  private void getLastFrameVelocity() {
    if (last6PoseFrames.size() < 2) {
      lastV = 0;
      lastVx = 0;
      lastVy = 0;
      lastWz = 0;
      return;
    }

    PoseFrame frame2 = last6PoseFrames.get(last6PoseFrames.size() - 1);
    PoseFrame frame1 = last6PoseFrames.get(last6PoseFrames.size() - 2);

    Pose3d pose2 = frame2.questPose3d();
    Pose3d pose1 = frame1.questPose3d();

    double dt = frame2.dataTimestamp() - frame1.dataTimestamp();

    lastVx = (pose2.getX() - pose1.getX()) / dt;
    lastVy = (pose2.getY() - pose1.getY()) / dt;
    lastV = Math.sqrt(lastVx * lastVx + lastVy * lastVy);

    lastWz = (pose2.getRotation().getZ() - pose1.getRotation().getZ()) / dt;
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
