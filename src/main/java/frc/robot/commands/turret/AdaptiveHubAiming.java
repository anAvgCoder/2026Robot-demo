package frc.robot.commands.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.ShotTable;
import frc.robot.subsystems.turret.ShotTable.ShotSetpoint;
import frc.robot.subsystems.turret.ShotTimeTable;
import frc.robot.subsystems.turret.hood.Hood;
import frc.robot.subsystems.turret.rotater.Rotater;
import frc.robot.subsystems.turret.rotater.RotaterConstants;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class AdaptiveHubAiming extends Command {
  private final Rotater rotaterRight;
  private final Hood hoodRight;
  private final Rotater rotaterLeft;
  private final Hood hoodLeft;
  private final Drive drive;
  private final boolean isBlue;
  private final ShotTable shotTable;

  private static final double TURRET_X_INCHES = 6.5;
  private static final double TURRET_Y_INCHES = 6.75;

  private static final int LOOKAHEAD_ITERS = 4;
  private static final double TOF_EPSILON_SEC = 0.005;
  private static final double MAX_LEAD_TIME_SEC = 2.0;
  private static final double SHOT_RELEASE_DELAY_SEC = 0.0;
  private static final double TOF_TABLE_MIN_M = 0.0;
  private static final double TOF_TABLE_MAX_M = 5.13;

  public AdaptiveHubAiming(
      Rotater rotaterRight,
      Hood hoodRight,
      Rotater rotaterLeft,
      Hood hoodLeft,
      Drive drive,
      boolean isBlueCheck,
      ShotTable shotTable) {
    this.rotaterRight = rotaterRight;
    this.hoodRight = hoodRight;
    this.rotaterLeft = rotaterLeft;
    this.hoodLeft = hoodLeft;
    this.drive = drive;
    this.shotTable = shotTable;
    this.isBlue = isBlueCheck;

    addRequirements(rotaterRight, hoodRight, rotaterLeft, hoodLeft);
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds robotRelativeSpeeds = drive.getRobotRelativeSpeeds();

    AimSolution rightSolution =
        solveAim(robotPose, robotRelativeSpeeds, true, RotaterConstants.turretRightAngleLocation);
    rotaterRight.setTurnPosition(rightSolution.turretAngleDeg);
    hoodRight.setHoodPosition(rightSolution.shotSetpoint.hoodPos());
    logAimSolution("Right", rightSolution);

    AimSolution leftSolution =
        solveAim(robotPose, robotRelativeSpeeds, false, RotaterConstants.turretLeftAngleLocation);
    rotaterLeft.setTurnPosition(leftSolution.turretAngleDeg);
    hoodLeft.setHoodPosition(leftSolution.shotSetpoint.hoodPos());
    logAimSolution("Left", leftSolution);
  }

  @Override
  public void end(boolean interrupted) {
    rotaterRight.setVoltage(0.0);
    rotaterLeft.setVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private AimSolution solveAim(
      Pose2d robotPose,
      ChassisSpeeds robotRelativeSpeeds,
      boolean isRightTurret,
      double turretMountAngleDeg) {

    Translation2d hubField = getHubTranslation();
    Translation2d turretOffsetRobot = getTurretOffsetRobot(isRightTurret);
    Translation2d turretOffsetField = turretOffsetRobot.rotateBy(robotPose.getRotation());
    Translation2d pivotFieldPosition = robotPose.getTranslation().plus(turretOffsetField);

    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());

    Translation2d pivotFieldVelocity =
        computePivotFieldVelocity(
            fieldRelativeSpeeds, turretOffsetField, robotRelativeSpeeds.omegaRadiansPerSecond);

    Translation2d releasePivotFieldPosition =
        addScaled(pivotFieldPosition, pivotFieldVelocity, SHOT_RELEASE_DELAY_SEC);
    double shotDistanceMeters = releasePivotFieldPosition.getDistance(hubField);
    ShotSetpoint shotSetpoint = this.shotTable.get(shotDistanceMeters);

    double tofSeconds = flightTimeSecondsSafe(shotDistanceMeters);
    Translation2d aimPointField = hubField;
    double aimPathDistanceMeters = shotDistanceMeters;

    for (int i = 0; i < LOOKAHEAD_ITERS; i++) {
      double leadTimeSeconds =
          MathUtil.clamp(SHOT_RELEASE_DELAY_SEC + tofSeconds, 0.0, MAX_LEAD_TIME_SEC);

      Translation2d newAimPointField = addScaled(hubField, pivotFieldVelocity, -leadTimeSeconds);
      double newAimPathDistanceMeters = pivotFieldPosition.getDistance(newAimPointField);
      double newTofSeconds = flightTimeSecondsSafe(newAimPathDistanceMeters);

      aimPointField = newAimPointField;
      aimPathDistanceMeters = newAimPathDistanceMeters;

      if (Math.abs(newTofSeconds - tofSeconds) < TOF_EPSILON_SEC) {
        tofSeconds = newTofSeconds;
        break;
      }

      tofSeconds = newTofSeconds;
    }

    double fieldAimAngleRad =
        Math.atan2(
            aimPointField.getY() - pivotFieldPosition.getY(),
            aimPointField.getX() - pivotFieldPosition.getX());

    double turretRelativeRad =
        MathUtil.angleModulus(fieldAimAngleRad - robotPose.getRotation().getRadians());
    turretRelativeRad =
        MathUtil.angleModulus(turretRelativeRad - Math.toRadians(turretMountAngleDeg));

    return new AimSolution(
        Math.toDegrees(turretRelativeRad),
        fieldAimAngleRad,
        shotDistanceMeters,
        aimPathDistanceMeters,
        tofSeconds,
        shotSetpoint,
        pivotFieldPosition,
        aimPointField,
        pivotFieldVelocity);
  }

  private Translation2d computePivotFieldVelocity(
      ChassisSpeeds fieldRelativeSpeeds,
      Translation2d turretOffsetField,
      double omegaRadiansPerSecond) {
    Translation2d centerFieldVelocity =
        new Translation2d(
            fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);

    Translation2d rotationalFieldVelocity =
        new Translation2d(
            -omegaRadiansPerSecond * turretOffsetField.getY(),
            omegaRadiansPerSecond * turretOffsetField.getX());

    return centerFieldVelocity.plus(rotationalFieldVelocity);
  }

  private Translation2d getTurretOffsetRobot(boolean isRightTurret) {
    return new Translation2d(
        Units.inchesToMeters(TURRET_X_INCHES),
        Units.inchesToMeters(isRightTurret ? -TURRET_Y_INCHES : TURRET_Y_INCHES));
  }

  private Translation2d getHubTranslation() {
    Pose3d hubPose = isBlue ? FieldConstants.BLUE_HUB_POSE3D : FieldConstants.RED_HUB_POSE3D;
    return new Translation2d(hubPose.getX(), hubPose.getY());
  }

  private Translation2d addScaled(
      Translation2d position, Translation2d velocity, double dtSeconds) {
    return new Translation2d(
        position.getX() + velocity.getX() * dtSeconds,
        position.getY() + velocity.getY() * dtSeconds);
  }

  private double flightTimeSecondsSafe(double distanceMeters) {
    double clampedDistance = MathUtil.clamp(distanceMeters, TOF_TABLE_MIN_M, TOF_TABLE_MAX_M);
    double tofSeconds = ShotTimeTable.getFlightTimeSeconds(clampedDistance);
    return MathUtil.clamp(tofSeconds, 0.0, MAX_LEAD_TIME_SEC);
  }

  private void logAimSolution(String key, AimSolution solution) {
    Logger.recordOutput("AimDebug/" + key + "/TOFSeconds", solution.tofSeconds);
    Logger.recordOutput("AimDebug/" + key + "/ShotDistanceMeters", solution.shotDistanceMeters);
    Logger.recordOutput(
        "AimDebug/" + key + "/AimPathDistanceMeters", solution.aimPathDistanceMeters);
    Logger.recordOutput("AimDebug/" + key + "/TurretAngleDeg", solution.turretAngleDeg);
    Logger.recordOutput(
        "AimDebug/" + key + "/PivotVelocityXMps", solution.pivotFieldVelocity.getX());
    Logger.recordOutput(
        "AimDebug/" + key + "/PivotVelocityYMps", solution.pivotFieldVelocity.getY());

    Logger.recordOutput(
        "AimDebug/" + key + "/PivotPose",
        new Pose2d(solution.pivotFieldPosition, new Rotation2d()));

    Logger.recordOutput(
        "AimDebug/" + key + "/AimPose",
        new Pose2d(solution.pivotFieldPosition, Rotation2d.fromRadians(solution.fieldAimAngleRad)));

    Logger.recordOutput(
        "AimDebug/" + key + "/VirtualTargetPose",
        new Pose2d(solution.aimPointField, new Rotation2d()));
  }

  private static final class AimSolution {
    final double turretAngleDeg;
    final double fieldAimAngleRad;
    final double shotDistanceMeters;
    final double aimPathDistanceMeters;
    final double tofSeconds;
    final ShotSetpoint shotSetpoint;
    final Translation2d pivotFieldPosition;
    final Translation2d aimPointField;
    final Translation2d pivotFieldVelocity;

    AimSolution(
        double turretAngleDeg,
        double fieldAimAngleRad,
        double shotDistanceMeters,
        double aimPathDistanceMeters,
        double tofSeconds,
        ShotSetpoint shotSetpoint,
        Translation2d pivotFieldPosition,
        Translation2d aimPointField,
        Translation2d pivotFieldVelocity) {
      this.turretAngleDeg = turretAngleDeg;
      this.fieldAimAngleRad = fieldAimAngleRad;
      this.shotDistanceMeters = shotDistanceMeters;
      this.aimPathDistanceMeters = aimPathDistanceMeters;
      this.tofSeconds = tofSeconds;
      this.shotSetpoint = shotSetpoint;
      this.pivotFieldPosition = pivotFieldPosition;
      this.aimPointField = aimPointField;
      this.pivotFieldVelocity = pivotFieldVelocity;
    }
  }
}
