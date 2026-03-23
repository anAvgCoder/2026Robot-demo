package frc.robot.commands.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

// TODO: adaptiveHubAiming include shooter motions in this too

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
  private static final double TURRET_Y_INCHES =
      6.75; // setting to zero to have them track together in separate solver below

  private static final int LOOKAHEAD_ITERS = 4;
  private static final double TOF_EPSILON_SEC = 0.005;
  private static final double MAX_LEAD_TIME_SEC = 2.0;
  private static final double SHOT_RELEASE_DELAY_SEC = 0.0;
  private static final double TOF_TABLE_MIN_M = 0.0;
  private static final double TOF_TABLE_MAX_M = 5.13;

  private static enum Target {
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

  Target targetChoice = Target.HUB;

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();

    //  we need to pick the target here
    //
    // determine where we are on field and set target accordingly
    // driver side HUB
    // mid and far field target outpost or depot

    if (robotPose.getX() < 3.8) {
      Logger.recordOutput("AimDebug/FieldZone", "driver side");

      targetChoice = Target.HUB;
    } else if (robotPose.getX() >= 3.8 && robotPose.getX() < 6) {
      Logger.recordOutput("AimDebug/FieldZone", "close trench zone");

      targetChoice = Target.NONE;
    } else if (robotPose.getX() >= 6) {

      // do left right of field for depot or outpost shooting

      Logger.recordOutput("AimDebug/FieldZone", "mid field");
      // mid field and farther

      // this may be oposite for red need to test or
      // does it really matter as it is just a zone
      if (robotPose.getY() < Units.inchesToMeters(FieldConstants.FIELD_WIDTH_INCHES / 2)) {

        targetChoice = Target.OUTPOST;
        Logger.recordOutput("AimDebug/FieldZone2", "outpost");
      } else {

        targetChoice = Target.DEPOT;
        Logger.recordOutput("AimDebug/FieldZone2", "depot");
      }
    }

    // need to add far trench zone here to protect the hoods

    //  add far driver zone

    Pose3d targetPose = getTargetPose(targetChoice);

    Logger.recordOutput("AimDebug/TargetPose", targetPose);

    //  solver right
    Pose3d turretPoseRight = getTurretPose(true);

    // need to have a target pose select for the distance calc
    double distRight = calculateHubDistance(turretPoseRight, targetPose);

    ShotSetpoint shotRight = shotTable.get(distRight);
    shooterRight.setVelocityRPM(shotRight.shooterSpeed());

    Logger.recordOutput("AimDebug/" + "Right" + "/ShooterDist", distRight);
    Logger.recordOutput("AimDebug/" + "Right" + "/ShooterTargetRPM", shotRight.shooterSpeed());

    // solver left
    Pose3d turretPoseLeft = getTurretPose(false);

    double distLeft = calculateHubDistance(turretPoseLeft, targetPose);

    ShotSetpoint shotLeft = shotTable.get(distLeft);
    shooterLeft.setVelocityRPM(shotLeft.shooterSpeed());

    Logger.recordOutput("AimDebug/" + "Left" + "/ShooterDist", distLeft);
    Logger.recordOutput("AimDebug/" + "Left" + "/ShooterTargetRPM", shotLeft.shooterSpeed());

    ChassisSpeeds robotRelativeSpeeds = drive.getRobotRelativeSpeeds();

    // central turret solver for rotation to keep turrets locked together
    Pose3d turretPoseCenter = getCenterTurretPose();

    Translation2d turretOffsetRobot = getTurretOffsetRobotCenter();

    AimSolution centerSolution =
        solveAim(
            robotPose,
            robotRelativeSpeeds,
            turretOffsetRobot,
            RotaterConstants.turretRightAngleLocation,
            targetPose);

    turretOffsetRobot = getTurretOffsetRobot(true);

    AimSolution rightSolution =
        solveAim(
            robotPose,
            robotRelativeSpeeds,
            turretOffsetRobot,
            RotaterConstants.turretRightAngleLocation,
            targetPose);

    hoodRight.setHoodPosition(rightSolution.shotSetpoint.hoodPos());
    logAimSolution("Right", rightSolution);

    turretOffsetRobot = getTurretOffsetRobot(false);

    AimSolution leftSolution =
        solveAim(
            robotPose,
            robotRelativeSpeeds,
            turretOffsetRobot,
            RotaterConstants.turretLeftAngleLocation,
            targetPose);

    hoodLeft.setHoodPosition(leftSolution.shotSetpoint.hoodPos());
    logAimSolution("Left", leftSolution);

    // to keep turn tracking together we dont want one to spin without
    // the other so we will use the central calc then check the delta against the
    // left and right to see if we need to limit the turn
    Logger.recordOutput("AimDebug/Master/rotatorLeft", leftSolution.turretAngleDeg);
    Logger.recordOutput("AimDebug/Master/rotatorRight", rightSolution.turretAngleDeg);
    Logger.recordOutput("AimDebug/Master/rotatorCenter", centerSolution.turretAngleDeg);

    double deltaRight = centerSolution.turretAngleDeg - rightSolution.turretAngleDeg;

    double deltaLeft = centerSolution.turretAngleDeg - leftSolution.turretAngleDeg;

    double deltaLeftToRight = rightSolution.turretAngleDeg - leftSolution.turretAngleDeg;

    Logger.recordOutput("AimDebug/Master/rotatorLeftDeltaCenter", deltaLeft);
    Logger.recordOutput("AimDebug/Master/rotatorRightDeltaCenter", deltaRight);
    Logger.recordOutput("AimDebug/Master/rotatorRightDeltaLeft", deltaLeftToRight);

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

  private Pose3d getTargetPose(Target targetChoice2) {

    Pose3d targetPose;

    if (targetChoice2 == Target.HUB) {

      targetPose = isBlue ? FieldConstants.BLUE_HUB_POSE3D : FieldConstants.RED_HUB_POSE3D;
    } else if (targetChoice2 == Target.DEPOT) {

      targetPose = isBlue ? FieldConstants.BLUE_DEPOT_POSE3D : FieldConstants.RED_DEPOT_POSE3D;
    } else if (targetChoice2 == Target.OUTPOST) {

      targetPose = isBlue ? FieldConstants.BLUE_OUTPOST_POSE3D : FieldConstants.RED_OUTPOST_POSE3D;
    } else {

      // none target will be pause - a center of field
      targetPose = isBlue ? FieldConstants.BLUE_HUB_POSE3D : FieldConstants.RED_HUB_POSE3D;
    }

    return targetPose;
  }

  @Override
  public void end(boolean interrupted) {
    rotaterRight.setVoltage(0.0);
    rotaterLeft.setVoltage(0.0);

    // set hoods down
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
      Pose3d target) {

    // Translation2d hubField = getHubTranslation();
    Translation2d hubField = getTargetTranslation(target);

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

  private Translation2d getTurretOffsetRobotCenter() {
    return new Translation2d(Units.inchesToMeters(TURRET_X_INCHES), Units.inchesToMeters(0));
  }

  private Translation2d getHubTranslation() {
    Pose3d hubPose = isBlue ? FieldConstants.BLUE_HUB_POSE3D : FieldConstants.RED_HUB_POSE3D;
    return new Translation2d(hubPose.getX(), hubPose.getY());
  }

  private Translation2d getTargetTranslation(Pose3d target) {

    return new Translation2d(target.getX(), target.getY());
  }

  private Translation2d getTargetTranslation() {

    Pose3d targetPose = isBlue ? FieldConstants.BLUE_HUB_POSE3D : FieldConstants.RED_HUB_POSE3D;
    return new Translation2d(targetPose.getX(), targetPose.getY());
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

  private Transform3d robotToTurret(boolean isRightTurret) {
    double xMeters = Units.inchesToMeters(TURRET_X_INCHES);
    double yMeters = Units.inchesToMeters(isRightTurret ? -TURRET_Y_INCHES : TURRET_Y_INCHES);
    return new Transform3d(xMeters, yMeters, 0.0, new Rotation3d());
  }

  private Transform3d robotToCenterTurret() {
    double xMeters = Units.inchesToMeters(TURRET_X_INCHES);
    double yMeters = Units.inchesToMeters(0);
    return new Transform3d(xMeters, yMeters, 0.0, new Rotation3d());
  }

  private Pose3d getTurretPose(boolean isRightTurret) {
    Pose3d robotPose = new Pose3d(drive.getPose());
    return robotPose.transformBy(robotToTurret(isRightTurret));
  }

  private Pose3d getCenterTurretPose() {
    Pose3d robotPose = new Pose3d(drive.getPose());
    return robotPose.transformBy(robotToCenterTurret());
  }

  public double calculateHubDistance(Pose3d turretPose, Pose3d target) {

    double dx = turretPose.getX() - target.getX();
    double dy = turretPose.getY() - target.getY();
    return Math.hypot(dx, dy);
  }

  private void logAimSolution(String key, AimSolution solution) {
    Logger.recordOutput("AimDebug/" + key + "/TOFSeconds", solution.tofSeconds);
    Logger.recordOutput("AimDebug/" + key + "/ShotDistanceMeters", solution.shotDistanceMeters);
    Logger.recordOutput(
        "AimDebug/" + key + "/AimPathDistanceMeters", solution.aimPathDistanceMeters);
    Logger.recordOutput("AimDebug/" + key + "/TurretAngleDeg", solution.turretAngleDeg);

    Logger.recordOutput("AimDebug/" + key + "/HoodPosition", solution.shotSetpoint.hoodPos());

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
