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
import frc.robot.subsystems.turret.shooter.Shooter;
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

  private final Shooter shooterRight;
  private final Shooter shooterLeft;

  private static final double TURRET_X_INCHES = 6.5;
  private static final double TURRET_Y_INCHES = 6.75;

  private static final int LOOKAHEAD_ITERS = 4;
  private static final double TOF_EPSILON_SEC = 0.005;
  private static final double MAX_LEAD_TIME_SEC = 2.0;
  private static final double SHOT_RELEASE_DELAY_SEC = 0.05; // ~50ms mechanical delay
  private static final double TOF_TABLE_MIN_M = 0.0;
  private static final double TOF_TABLE_MAX_M = 15.0; // was 5.13, table goes to 15

  private enum Target {
    HUB,
    OUTPOST,
    DEPOT,
    NONE
  }

  public AdaptiveHubAiming(
      Shooter shooterRight,
      Rotater rotaterRight,
      Hood hoodRight,
      Shooter shooterLeft,
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
    this.shooterRight = shooterRight;
    this.shooterLeft = shooterLeft;

    addRequirements(rotaterRight, hoodRight, rotaterLeft, hoodLeft);
    addRequirements(shooterRight, shooterLeft);
  }

  private Target targetChoice = Target.HUB;

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds robotRelativeSpeeds = drive.getRobotRelativeSpeeds();

    // Pick target based on field zone
    if (robotPose.getX() < 3.8) {
      targetChoice = Target.HUB;
      Logger.recordOutput("AimDebug/FieldZone", "driver side");
    } else if (robotPose.getX() < 6) {
      targetChoice = Target.NONE;
      Logger.recordOutput("AimDebug/FieldZone", "close trench zone");
    } else {
      if (robotPose.getY() < Units.inchesToMeters(FieldConstants.FIELD_WIDTH_INCHES / 2)) {
        targetChoice = Target.OUTPOST;
        Logger.recordOutput("AimDebug/FieldZone", "outpost");
      } else {
        targetChoice = Target.DEPOT;
        Logger.recordOutput("AimDebug/FieldZone", "depot");
      }
    }

    Pose3d targetPose = getTargetPose(targetChoice);
    Translation2d targetField = new Translation2d(targetPose.getX(), targetPose.getY());
    Logger.recordOutput("AimDebug/TargetPose", targetPose);

    // Solve right turret — hood, shooter, and angle all from one solution
    AimSolution rightSolution =
        solveAim(
            robotPose,
            robotRelativeSpeeds,
            getTurretOffsetRobot(true),
            RotaterConstants.turretRightAngleLocation,
            targetField);

    hoodRight.setHoodPosition(rightSolution.shotSetpoint.hoodPos());
    shooterRight.setVelocityRPM(rightSolution.shotSetpoint.shooterSpeed());
    logAimSolution("Right", rightSolution);

    // Solve left turret
    AimSolution leftSolution =
        solveAim(
            robotPose,
            robotRelativeSpeeds,
            getTurretOffsetRobot(false),
            RotaterConstants.turretLeftAngleLocation,
            targetField);

    hoodLeft.setHoodPosition(leftSolution.shotSetpoint.hoodPos());
    shooterLeft.setVelocityRPM(leftSolution.shotSetpoint.shooterSpeed());
    logAimSolution("Left", leftSolution);

    // Solve center (virtual turret at y=0) for rotation coordination
    AimSolution centerSolution =
        solveAim(
            robotPose,
            robotRelativeSpeeds,
            getTurretOffsetRobotCenter(),
            RotaterConstants.turretRightAngleLocation,
            targetField);

    // Turret rotation — use center when left/right have a wrap-around delta
    double deltaLeftToRight = rightSolution.turretAngleDeg - leftSolution.turretAngleDeg;

    Logger.recordOutput("AimDebug/Master/rotatorLeft", leftSolution.turretAngleDeg);
    Logger.recordOutput("AimDebug/Master/rotatorRight", rightSolution.turretAngleDeg);
    Logger.recordOutput("AimDebug/Master/rotatorCenter", centerSolution.turretAngleDeg);
    Logger.recordOutput("AimDebug/Master/deltaLeftToRight", deltaLeftToRight);

    if (Math.abs(deltaLeftToRight) > 300) {
      Logger.recordOutput("AimDebug/Master/UsingIndividual", false);
      rotaterLeft.setTurnPosition(centerSolution.turretAngleDeg);
      rotaterRight.setTurnPosition(centerSolution.turretAngleDeg);
    } else {
      Logger.recordOutput("AimDebug/Master/UsingIndividual", true);
      rotaterLeft.setTurnPosition(leftSolution.turretAngleDeg);
      rotaterRight.setTurnPosition(rightSolution.turretAngleDeg);
    }
  }

  private Pose3d getTargetPose(Target target) {
    switch (target) {
      case DEPOT:
        return isBlue ? FieldConstants.BLUE_DEPOT_POSE3D : FieldConstants.RED_DEPOT_POSE3D;
      case OUTPOST:
        return isBlue ? FieldConstants.BLUE_OUTPOST_POSE3D : FieldConstants.RED_OUTPOST_POSE3D;
      case HUB:
      case NONE:
      default:
        return isBlue ? FieldConstants.BLUE_HUB_POSE3D : FieldConstants.RED_HUB_POSE3D;
    }
  }

  @Override
  public void end(boolean interrupted) {
    rotaterRight.setVoltage(0.0);
    rotaterLeft.setVoltage(0.0);
    hoodLeft.setHoodPosition(0);
    hoodRight.setHoodPosition(0);
    shooterRight.stop();
    shooterLeft.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private AimSolution solveAim(
      Pose2d robotPose,
      ChassisSpeeds robotRelativeSpeeds,
      Translation2d turretOffsetRobot,
      double turretMountAngleDeg,
      Translation2d targetField) {

    Translation2d turretOffsetField = turretOffsetRobot.rotateBy(robotPose.getRotation());
    Translation2d pivotFieldPosition = robotPose.getTranslation().plus(turretOffsetField);

    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());

    Translation2d pivotFieldVelocity =
        computePivotFieldVelocity(
            fieldRelativeSpeeds, turretOffsetField, robotRelativeSpeeds.omegaRadiansPerSecond);

    // Where the turret will be when the ball actually exits (accounts for mechanical delay)
    Translation2d releasePivotFieldPosition =
        addScaled(pivotFieldPosition, pivotFieldVelocity, SHOT_RELEASE_DELAY_SEC);
    double shotDistanceMeters = releasePivotFieldPosition.getDistance(targetField);

    double tofSeconds = flightTimeSecondsSafe(shotDistanceMeters);
    Translation2d aimPointField = targetField;
    double aimPathDistanceMeters = shotDistanceMeters;

    // Iterative lead solver — converge aim point with time-of-flight
    for (int i = 0; i < LOOKAHEAD_ITERS; i++) {
      double leadTimeSeconds =
          MathUtil.clamp(SHOT_RELEASE_DELAY_SEC + tofSeconds, 0.0, MAX_LEAD_TIME_SEC);

      Translation2d newAimPointField = addScaled(targetField, pivotFieldVelocity, -leadTimeSeconds);
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

    // Use the lead-compensated distance for BOTH hood and shooter RPM
    ShotSetpoint shotSetpoint = shotTable.get(aimPathDistanceMeters);

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
            -omegaRadiansPerSecond * turretOffsetField.getY() * 0.5,
            omegaRadiansPerSecond * turretOffsetField.getX() * 0.5);
    return centerFieldVelocity.plus(rotationalFieldVelocity);
  }

  private Translation2d getTurretOffsetRobot(boolean isRightTurret) {
    return new Translation2d(
        Units.inchesToMeters(TURRET_X_INCHES),
        Units.inchesToMeters(isRightTurret ? -TURRET_Y_INCHES : TURRET_Y_INCHES));
  }

  private Translation2d getTurretOffsetRobotCenter() {
    return new Translation2d(Units.inchesToMeters(TURRET_X_INCHES), 0.0);
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
    Logger.recordOutput("AimDebug/" + key + "/ShotDistMeters", solution.shotDistanceMeters);
    Logger.recordOutput("AimDebug/" + key + "/AimPathDistMeters", solution.aimPathDistanceMeters);
    Logger.recordOutput("AimDebug/" + key + "/TurretAngleDeg", solution.turretAngleDeg);
    Logger.recordOutput("AimDebug/" + key + "/HoodPos", solution.shotSetpoint.hoodPos());
    Logger.recordOutput("AimDebug/" + key + "/ShooterRPM", solution.shotSetpoint.shooterSpeed());
    Logger.recordOutput("AimDebug/" + key + "/PivotVelX", solution.pivotFieldVelocity.getX());
    Logger.recordOutput("AimDebug/" + key + "/PivotVelY", solution.pivotFieldVelocity.getY());

    Logger.recordOutput(
        "AimDebug/" + key + "/PivotPose",
        new Pose2d(solution.pivotFieldPosition, new Rotation2d()));
    Logger.recordOutput(
        "AimDebug/" + key + "/AimPose",
        new Pose2d(solution.pivotFieldPosition, Rotation2d.fromRadians(solution.fieldAimAngleRad)));
    Logger.recordOutput(
        "AimDebug/" + key + "/VirtualTarget", new Pose2d(solution.aimPointField, new Rotation2d()));
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
