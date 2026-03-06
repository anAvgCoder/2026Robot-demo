package frc.robot.commands.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.ShotTable;
import frc.robot.subsystems.turret.ShotTable.ShotSetpoint;
import frc.robot.subsystems.turret.ShotTimeTable;
import frc.robot.subsystems.turret.hood.Hood;
import frc.robot.subsystems.turret.hood.HoodIO;
import frc.robot.subsystems.turret.rotater.Rotater;
import frc.robot.subsystems.turret.rotater.RotaterConstants;
import frc.robot.subsystems.turret.rotater.RotaterIO;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;


public class AdaptiveHubAiming extends Command {
  private final RotaterIO rotaterIORight;
  private final HoodIO hoodIORight;

  private final RotaterIO rotaterIOLeft;
  private final HoodIO hoodIOLeft;

  private final Drive drive;
  private final boolean isBlue;

  private int runCounter;

  private static final double TURRET_X_INCHES = 6.5;
  private static final double TURRET_Y_INCHES = 6.75;

  private static final int LOOKAHEAD_ITERS = 3;
  private static final double TOF_EPSILON_SEC = 0.02;

  private static final double TOF_TABLE_MIN_M = 0.0;
  private static final double TOF_TABLE_MAX_M = 5.13;

  public AdaptiveHubAiming(
      Rotater rotaterRight,
      Hood hoodRight,
      Rotater rotaterLeft,
      Hood hoodLeft,
      Drive drive,
      boolean isBlueCheck) {

    this.rotaterIORight = rotaterRight.getIO();
    this.hoodIORight = hoodRight.getIO();

    this.rotaterIOLeft = rotaterLeft.getIO();
    this.hoodIOLeft = hoodLeft.getIO();

    this.drive = drive;
    this.isBlue = isBlueCheck;

    addRequirements(rotaterRight, hoodRight, rotaterLeft, hoodLeft);
  }

  @Override
  public void initialize() {
    runCounter = 0;
  }

  @Override
  public void execute() {
    Pose2d basePose = drive.getPose();
    ChassisSpeeds robotSpeeds = drive.getRobotRelativeSpeeds();

    Pose3d turretPoseRight = predictTurretPose(basePose, robotSpeeds, true);
    double rightDeg =
        calculateTurretDegreesRobotRelative(
            turretPoseRight, RotaterConstants.turretRightAngleLocation);
    rotaterIORight.setTurnPosition(rightDeg);

    double distRight = hubDistance(turretPoseRight);
    ShotSetpoint spRight = ShotTable.get(distRight);
    hoodIORight.setHoodPosition(spRight.hoodPos());

    // Left turret
    Pose3d turretPoseLeft = predictTurretPose(basePose, robotSpeeds, false);
    double leftDeg =
        calculateTurretDegreesRobotRelative(
            turretPoseLeft, RotaterConstants.turretLeftAngleLocation);
    rotaterIOLeft.setTurnPosition(leftDeg);

    double distLeft = hubDistance(turretPoseLeft);
    ShotSetpoint spLeft = ShotTable.get(distLeft);
    hoodIOLeft.setHoodPosition(spLeft.hoodPos());

    runCounter++;
    if (runCounter > 24) {
      runCounter = 24;
    }
  }

  @Override
  public void end(boolean interrupted) {
    rotaterIORight.setVoltage(0.0);
    rotaterIOLeft.setVoltage(0.0);

    hoodIORight.setVoltage(0.0);
    hoodIOLeft.setVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private Transform3d robotToTurret(boolean isRightTurret) {
    double xMeters = Units.inchesToMeters(TURRET_X_INCHES);
    double yMeters = Units.inchesToMeters(isRightTurret ? -TURRET_Y_INCHES : TURRET_Y_INCHES);
    return new Transform3d(xMeters, yMeters, 0.0, new Rotation3d());
  }

  private Pose2d predictRobotPose(Pose2d basePose, ChassisSpeeds robotSpeeds, double dtSec) {
    Twist2d twist =
        new Twist2d(
            robotSpeeds.vxMetersPerSecond * dtSec,
            robotSpeeds.vyMetersPerSecond * dtSec,
            robotSpeeds.omegaRadiansPerSecond * dtSec);
    return basePose.exp(twist);
  }

  private Pose3d predictTurretPose(
    Pose2d basePose, ChassisSpeeds robotSpeeds, boolean isRightTurret) {
    Pose2d predictedPose = basePose;

    Pose3d turretPose = new Pose3d(predictedPose).transformBy(robotToTurret(isRightTurret));
    double dist = hubDistance(turretPose);
    double tof = flightTimeSecondsSafe(dist);

    for (int i = 0; i < LOOKAHEAD_ITERS; i++) {
      Pose2d newPredictedPose = predictRobotPose(basePose, robotSpeeds, tof);
      Pose3d newTurretPose = new Pose3d(newPredictedPose).transformBy(robotToTurret(isRightTurret));

      double newDist = hubDistance(newTurretPose);
      double newTof = flightTimeSecondsSafe(newDist);

      if (Math.abs(newTof - tof) < TOF_EPSILON_SEC) {
        predictedPose = newPredictedPose;
        turretPose = newTurretPose;
        dist = newDist;
        tof = newTof;
        break;
      }

      predictedPose = newPredictedPose;
      turretPose = newTurretPose;
      dist = newDist;
      tof = newTof;
    }

    Logger.recordOutput("AimDebug/PredictedTOF", tof);
    Logger.recordOutput("AimDebug/PredictedDistance", dist);

    return turretPose;
  }

  private double flightTimeSecondsSafe(double distanceMeters) {
    double clamped = MathUtil.clamp(distanceMeters, TOF_TABLE_MIN_M, TOF_TABLE_MAX_M);
    double tof = ShotTimeTable.getFlightTimeSeconds(clamped);
    return MathUtil.clamp(tof, 0.0, 2.0);
  }

  public double calculateTurretDegreesRobotRelative(Pose3d turretPose, double turretMountAngleDeg) {
    Pose3d hubPose = isBlue ? FieldConstants.BLUE_HUB_POSE3D : FieldConstants.RED_HUB_POSE3D;

    double dx = hubPose.getX() - turretPose.getX();
    double dy = hubPose.getY() - turretPose.getY();

    double fieldAngleRad = Math.atan2(dy, dx);

    double robotYawRad = turretPose.getRotation().getZ();

    double turretRelativeRad = MathUtil.angleModulus(fieldAngleRad - robotYawRad);

    turretRelativeRad =
        MathUtil.angleModulus(turretRelativeRad - Math.toRadians(turretMountAngleDeg));

    double aimFieldRad = robotYawRad + turretRelativeRad;
    Logger.recordOutput(
        "AimDebug/TurretAimPose",
        new Pose2d(turretPose.getX(), turretPose.getY(), new Rotation2d(aimFieldRad)));
    Logger.recordOutput(
        "AimDebug/TurretToHubPose",
        new Pose2d(turretPose.getX(), turretPose.getY(), new Rotation2d(fieldAngleRad)));

    return Math.toDegrees(turretRelativeRad);
  }

  public double hubDistance(Pose3d turretPose) {
    Pose3d hubPose = isBlue ? FieldConstants.BLUE_HUB_POSE3D : FieldConstants.RED_HUB_POSE3D;
    double dx = turretPose.getX() - hubPose.getX();
    double dy = turretPose.getY() - hubPose.getY();
    return Math.hypot(dx, dy);
  }
}
