// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.questnav.QuestNavConstants;
import frc.robot.subsystems.questnav.QuestNavSensor;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();

  // How tightly to trust Quest pose measurements when fusing into the estimator.
  // These are intentionally tight (2 cm XY, ~2 deg heading) because the Quest
  // is our primary localisation source.
  private static final Matrix<N3, N1> QUEST_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.035);

  private final QuestNavSensor quest;
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

  private Rotation2d rawGyroRotation = Rotation2d.kZero;

  private final SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  private double speedScale = DriveConstants.normalSpeedFactor;
  private double currentSpeedScale = speedScale;

  public Drive(
      GyroIO gyroIO,
      QuestNavSensor quest,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {

    this.gyroIO = gyroIO;
    this.quest = quest;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    SparkOdometryThread.getInstance().start();

    AutoBuilder.configure(
        this::getPose,
        this::resetPose2d,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) ->
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  private int loopCounterSpeed = 0;
  private double deltaPerInterval = 0;
  private boolean firstTime = true;
  private boolean overrideQuestForCamera = false;

  @Override
  public void periodic() {

    Logger.recordOutput("Swerve/Speed", speedScale);

    if (currentSpeedScale != speedScale) {
      if (currentSpeedScale > speedScale) {
        currentSpeedScale = speedScale;
        loopCounterSpeed = 0;
        firstTime = true;
      } else {
        double delta = Math.abs(currentSpeedScale - speedScale);
        if (loopCounterSpeed < 25) {
          if (firstTime) {
            deltaPerInterval = delta / 25;
          }
          currentSpeedScale = currentSpeedScale + deltaPerInterval;
        } else {
          currentSpeedScale = speedScale;
          loopCounterSpeed = 0;
          firstTime = true;
        }
      }
    }

    quest.runPeriodicUpdates();

    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    odometryLock.lock();
    try {
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
      for (var module : modules) {
        module.periodic();
      }

      double[] sampleTimestamps = modules[0].getOdometryTimestamps();
      int sampleCount = sampleTimestamps.length;

      for (int i = 0; i < sampleCount; i++) {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
          modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
          moduleDeltas[moduleIndex] =
              new SwerveModulePosition(
                  modulePositions[moduleIndex].distanceMeters
                      - lastModulePositions[moduleIndex].distanceMeters,
                  modulePositions[moduleIndex].angle);
          lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }

        if (gyroInputs.connected) {
          rawGyroRotation = gyroInputs.odometryYawPositions[i];
        } else {
          Twist2d twist = kinematics.toTwist2d(moduleDeltas);
          rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }

        poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
      }

      if (quest.isWorking()) {
        if (overrideQuestForCamera == false) {
          poseEstimator.addVisionMeasurement(
              quest.getRobotPose(), Timer.getFPGATimestamp(), QUEST_STD_DEVS);
        }
      }

    } finally {
      odometryLock.unlock();
    }

    Logger.recordOutput("QuestNav/isWorking", quest.isWorking());
    Logger.recordOutput("QuestNav/overrideToCamera", overrideQuestForCamera);
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  public void switchToCamera() {

    overrideQuestForCamera = true;
    System.out.println("switching to cameras from quest");
  }

  public void switchToQuest() {

    overrideQuestForCamera = false;
    System.out.println("switching to quest from cameras");
  }

  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    odometryLock.lock();
    try {
      return kinematics.toChassisSpeeds(getModuleStates());
    } finally {
      odometryLock.unlock();
    }
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    odometryLock.lock();
    try {
      return kinematics.toChassisSpeeds(getModuleStates());
    } finally {
      odometryLock.unlock();
    }
  }

  public Rotation2d getRawGyroRotation() {
    odometryLock.lock();
    try {
      return rawGyroRotation;
    } finally {
      odometryLock.unlock();
    }
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    ChassisSpeeds robot = getRobotRelativeSpeeds();
    Rotation2d heading = getRawGyroRotation();
    return ChassisSpeeds.fromRobotRelativeSpeeds(robot, heading);
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    odometryLock.lock();
    try {
      return poseEstimator.getEstimatedPosition();
    } finally {
      odometryLock.unlock();
    }
  }

  @AutoLogOutput(key = "Odometry/subsystemPosePoseEstimator")
  private Pose2d subsystemPose() {
    return getPose();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void setPose(Pose3d robotPose) {
    odometryLock.lock();
    try {
      Pose2d robotPose2d = robotPose.toPose2d();

      rawGyroRotation = robotPose2d.getRotation();
      gyroIO.setYaw(rawGyroRotation);

      quest.zeroQuestPose(robotPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST));

      poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), robotPose2d);
    } finally {
      odometryLock.unlock();
    }
  }

  public void resetPose2d(Pose2d pose) {
    odometryLock.lock();
    try {
      poseEstimator.resetPose(pose);
    } finally {
      odometryLock.unlock();
    }
  }

  public void resetQuestPose(Pose3d pose) {
    quest.zeroQuestPose(pose);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {

    if (quest.isWorking()) {
      if (overrideQuestForCamera == false) {
        return;
      }
    }

    odometryLock.lock();
    try {
      poseEstimator.addVisionMeasurement(
          visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    } finally {
      odometryLock.unlock();
    }
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec * currentSpeedScale;
  }

  public double getMaxAngularSpeedRadPerSec() {
    return (maxSpeedMetersPerSec / driveBaseRadius) * speedScale;
  }

  public QuestNavSensor getQuestNavSensor() {
    return quest;
  }

  public void setSpeedIntake() {
    speedScale = DriveConstants.intakeSpeedFactor;
  }

  public void setSpeedFull() {
    speedScale = DriveConstants.fullSpeedFactor;
  }

  public void setSpeedNormal() {
    speedScale = DriveConstants.normalSpeedFactor;
  }
}
