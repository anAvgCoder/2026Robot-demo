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

  // Robot-frame turret pivot offsets from robot center
  private static final double TURRET_X_INCHES = 6.5;
  private static final double TURRET_Y_INCHES = 6.75;

  // Lead-solver tuning
  private static final int    LOOKAHEAD_ITERS       = 6;
  private static final double TOF_EPSILON_SEC        = 0.001; // convergence threshold
  private static final double MAX_LEAD_TIME_SEC      = 2.0;
  private static final double SHOT_RELEASE_DELAY_SEC = 0.05;  // ~50 ms mechanical lag
  private static final double TOF_TABLE_MIN_M        = 0.0;
  private static final double TOF_TABLE_MAX_M        = 15.0;

  // If left/right turret angles differ by more than this, fall back to center solution
  private static final double WRAP_AROUND_THRESHOLD_DEG = 300.0;

  private enum Target { HUB, OUTPOST, DEPOT, NONE }

  private Target targetChoice = Target.HUB;

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
    this.hoodRight    = hoodRight;
    this.rotaterLeft  = rotaterLeft;
    this.hoodLeft     = hoodLeft;
    this.drive        = drive;
    this.shotTable    = shotTable;
    this.isBlue       = isBlueCheck;
    this.shooterRight = shooterRight;
    this.shooterLeft  = shooterLeft;

    addRequirements(rotaterRight, hoodRight, rotaterLeft, hoodLeft);
    addRequirements(shooterRight, shooterLeft);
  }

  @Override
  public void execute() {
    Pose2d robotPose            = drive.getPose();
    ChassisSpeeds robotSpeeds   = drive.getRobotRelativeSpeeds();

    updateTargetChoice(robotPose);

    Pose3d        targetPose  = getTargetPose(targetChoice);
    Translation2d targetField = new Translation2d(targetPose.getX(), targetPose.getY());
    Logger.recordOutput("AimDebug/TargetPose", targetPose);

    // ── Solve all three pivot positions ─────────────────────────────────────
    AimSolution rightSolution = solveAim(
        robotPose, robotSpeeds,
        getTurretOffsetRobot(true),
        RotaterConstants.turretRightAngleLocation,
        targetField);

    AimSolution leftSolution = solveAim(
        robotPose, robotSpeeds,
        getTurretOffsetRobot(false),
        RotaterConstants.turretLeftAngleLocation,
        targetField);

    // Virtual center turret — used to resolve wrap-around discontinuities
    AimSolution centerSolution = solveAim(
        robotPose, robotSpeeds,
        getTurretOffsetRobotCenter(),
        RotaterConstants.turretRightAngleLocation,
        targetField);

    // ── Apply hood and shooter setpoints ────────────────────────────────────
    hoodRight.setHoodPosition(rightSolution.shotSetpoint.hoodPos());
    shooterRight.setVelocityRPM(rightSolution.shotSetpoint.shooterSpeed());

    hoodLeft.setHoodPosition(leftSolution.shotSetpoint.hoodPos());
    shooterLeft.setVelocityRPM(leftSolution.shotSetpoint.shooterSpeed());

    // ── Apply turret rotation ────────────────────────────────────────────────
    // If left and right angles are on opposite sides of the ±180° wrap boundary
    // their individual solutions will be numerically far apart even though they
    // physically agree.  Fall back to the center solution in that case.
    double deltaLeftToRight = rightSolution.turretAngleDeg - leftSolution.turretAngleDeg;

    Logger.recordOutput("AimDebug/Master/rotatorLeft",      leftSolution.turretAngleDeg);
    Logger.recordOutput("AimDebug/Master/rotatorRight",     rightSolution.turretAngleDeg);
    Logger.recordOutput("AimDebug/Master/rotatorCenter",    centerSolution.turretAngleDeg);
    Logger.recordOutput("AimDebug/Master/deltaLeftToRight", deltaLeftToRight);

    if (Math.abs(deltaLeftToRight) > WRAP_AROUND_THRESHOLD_DEG) {
      Logger.recordOutput("AimDebug/Master/UsingIndividual", false);
      rotaterLeft.setTurnPosition(centerSolution.turretAngleDeg);
      rotaterRight.setTurnPosition(centerSolution.turretAngleDeg);
    } else {
      Logger.recordOutput("AimDebug/Master/UsingIndividual", true);
      rotaterLeft.setTurnPosition(leftSolution.turretAngleDeg);
      rotaterRight.setTurnPosition(rightSolution.turretAngleDeg);
    }

    logAimSolution("Right",  rightSolution);
    logAimSolution("Left",   leftSolution);
    logAimSolution("Center", centerSolution);
  }

  // ── Target selection ────────────────────────────────────────────────────────

  private void updateTargetChoice(Pose2d robotPose) {
    if (robotPose.getX() < Units.inchesToMeters(165)) {
      targetChoice = Target.HUB;
      Logger.recordOutput("AimDebug/FieldZone", "driver side");
    } else if (robotPose.getX() < Units.inchesToMeters(200)) {
      targetChoice = Target.NONE;
      Logger.recordOutput("AimDebug/FieldZone", "close trench zone");
    } else if (robotPose.getY() < Units.inchesToMeters(FieldConstants.FIELD_WIDTH_INCHES / 2)) {
      targetChoice = Target.OUTPOST;
      Logger.recordOutput("AimDebug/FieldZone", "outpost");
    } else {
      targetChoice = Target.DEPOT;
      Logger.recordOutput("AimDebug/FieldZone", "depot");
    }
  }

  private Pose3d getTargetPose(Target target) {
    switch (target) {
      case DEPOT:
        return isBlue ? FieldConstants.BLUE_DEPOT_POSE3D   : FieldConstants.RED_DEPOT_POSE3D;
      case OUTPOST:
        return isBlue ? FieldConstants.BLUE_OUTPOST_POSE3D : FieldConstants.RED_OUTPOST_POSE3D;
      case HUB:
      case NONE:
      default:
        return isBlue ? FieldConstants.BLUE_HUB_POSE3D     : FieldConstants.RED_HUB_POSE3D;
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

  // ── Core aiming solver ──────────────────────────────────────────────────────

  /**
   * Computes the full aiming solution for one turret pivot.
   *
   * <p><b>Physics summary:</b>
   * <ol>
   *   <li>The pivot's field-frame velocity is the robot's translational velocity
   *       plus the tangential velocity due to rotation: {@code v_pivot = v_center + ω × r}.
   *       Both terms are used at full magnitude — there is no fractional scaling.
   *   <li>The ball is not released instantly.  During {@code SHOT_RELEASE_DELAY_SEC} the
   *       pivot drifts to {@code releasePivotFieldPosition}, which is the true shot origin.
   *   <li>Because the shooter imparts velocity in the robot frame, the ball inherits
   *       the pivot's field-frame velocity.  To hit a stationary target the turret must
   *       aim at a virtual point {@code aimPointField = targetField - v_pivot * tof},
   *       found by iterating over the time-of-flight until convergence.
   *   <li>All distance lookups (shot table and TOF table) use the lead-corrected distance
   *       from {@code releasePivotFieldPosition} to {@code aimPointField}.
   * </ol>
   */
  private AimSolution solveAim(
      Pose2d robotPose,
      ChassisSpeeds robotRelativeSpeeds,
      Translation2d turretOffsetRobot,
      double turretMountAngleDeg,
      Translation2d targetField) {

    // ── 1. Pivot position and velocity in the field frame ───────────────────
    Translation2d turretOffsetField = turretOffsetRobot.rotateBy(robotPose.getRotation());
    Translation2d pivotFieldPosition = robotPose.getTranslation().plus(turretOffsetField);

    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());

    Translation2d pivotFieldVelocity = computePivotFieldVelocity(
        fieldSpeeds, turretOffsetField, robotRelativeSpeeds.omegaRadiansPerSecond);

    // ── 2. Advance pivot by the mechanical release delay ────────────────────
    Translation2d releasePivotFieldPosition =
        addScaled(pivotFieldPosition, pivotFieldVelocity, SHOT_RELEASE_DELAY_SEC);

    // ── 3. Iterative lead solver ─────────────────────────────────────────────
    // Seed: straight-line distance from the release position to the raw target.
    double        tofSeconds         = flightTimeSecondsSafe(releasePivotFieldPosition.getDistance(targetField));
    Translation2d aimPointField      = targetField;
    double        aimPathDistMeters  = releasePivotFieldPosition.getDistance(targetField);

    for (int i = 0; i < LOOKAHEAD_ITERS; i++) {
      // Virtual aim point: where the ball must be launched (in field frame) so that,
      // after the ball inherits v_pivot and travels for tofSeconds, it reaches targetField.
      //   targetField = releasePivot + (v_aim_robot + v_pivot) * tof
      //   => aim toward: targetField - v_pivot * tof
      Translation2d newAimPoint      = addScaled(targetField, pivotFieldVelocity, -tofSeconds);
      double        newAimPathDist   = releasePivotFieldPosition.getDistance(newAimPoint);
      double        newTof           = flightTimeSecondsSafe(newAimPathDist);

      aimPointField     = newAimPoint;
      aimPathDistMeters = newAimPathDist;

      if (Math.abs(newTof - tofSeconds) < TOF_EPSILON_SEC) {
        tofSeconds = newTof;
        break;
      }
      tofSeconds = newTof;
    }

    // ── 4. Shot table lookup using lead-corrected distance ───────────────────
    ShotSetpoint shotSetpoint = shotTable.get(aimPathDistMeters);

    // Raw distance (no lead) for logging only
    double shotDistanceMeters = releasePivotFieldPosition.getDistance(targetField);

    // ── 5. Turret angle ──────────────────────────────────────────────────────
    // Aim angle is from the *release* position toward the virtual aim point.
    double fieldAimAngleRad = Math.atan2(
        aimPointField.getY() - releasePivotFieldPosition.getY(),
        aimPointField.getX() - releasePivotFieldPosition.getX());

    // Subtract robot heading, then subtract the turret's mount offset
    double turretRelativeRad =
        MathUtil.angleModulus(fieldAimAngleRad - robotPose.getRotation().getRadians());
    turretRelativeRad =
        MathUtil.angleModulus(turretRelativeRad - Math.toRadians(turretMountAngleDeg));

    return new AimSolution(
        Math.toDegrees(turretRelativeRad),
        fieldAimAngleRad,
        shotDistanceMeters,
        aimPathDistMeters,
        tofSeconds,
        shotSetpoint,
        pivotFieldPosition,
        releasePivotFieldPosition,
        aimPointField,
        pivotFieldVelocity);
  }

  // ── Helpers ─────────────────────────────────────────────────────────────────

  /**
   * Field-frame velocity of a turret pivot using the full rigid-body formula.
   *
   * <pre>
   *   v_pivot = v_center + ω × r
   *   vx = vx_field − ω · ry
   *   vy = vy_field + ω · rx
   * </pre>
   *
   * There is intentionally no fractional scale on the rotational term; omitting it
   * caused systematic angular errors proportional to robot spin rate and turret offset.
   */
  private Translation2d computePivotFieldVelocity(
      ChassisSpeeds fieldRelativeSpeeds,
      Translation2d turretOffsetField,
      double omegaRadiansPerSecond) {
    return new Translation2d(
        fieldRelativeSpeeds.vxMetersPerSecond - omegaRadiansPerSecond * turretOffsetField.getY(),
        fieldRelativeSpeeds.vyMetersPerSecond + omegaRadiansPerSecond * turretOffsetField.getX());
  }

  private Translation2d getTurretOffsetRobot(boolean isRightTurret) {
    return new Translation2d(
        Units.inchesToMeters(TURRET_X_INCHES),
        Units.inchesToMeters(isRightTurret ? -TURRET_Y_INCHES : TURRET_Y_INCHES));
  }

  private Translation2d getTurretOffsetRobotCenter() {
    return new Translation2d(Units.inchesToMeters(TURRET_X_INCHES), 0.0);
  }

  /** Returns {@code base + delta * scale}. */
  private Translation2d addScaled(Translation2d base, Translation2d delta, double scale) {
    return new Translation2d(
        base.getX() + delta.getX() * scale,
        base.getY() + delta.getY() * scale);
  }

  private double flightTimeSecondsSafe(double distanceMeters) {
    double clamped = MathUtil.clamp(distanceMeters, TOF_TABLE_MIN_M, TOF_TABLE_MAX_M);
    return MathUtil.clamp(ShotTimeTable.getFlightTimeSeconds(clamped), 0.0, MAX_LEAD_TIME_SEC);
  }

  private void logAimSolution(String key, AimSolution s) {
    Logger.recordOutput("AimDebug/" + key + "/TOFSeconds",        s.tofSeconds);
    Logger.recordOutput("AimDebug/" + key + "/ShotDistMeters",    s.shotDistanceMeters);
    Logger.recordOutput("AimDebug/" + key + "/AimPathDistMeters", s.aimPathDistanceMeters);
    Logger.recordOutput("AimDebug/" + key + "/TurretAngleDeg",    s.turretAngleDeg);
    Logger.recordOutput("AimDebug/" + key + "/HoodPos",           s.shotSetpoint.hoodPos());
    Logger.recordOutput("AimDebug/" + key + "/ShooterRPM",        s.shotSetpoint.shooterSpeed());
    Logger.recordOutput("AimDebug/" + key + "/PivotVelX",         s.pivotFieldVelocity.getX());
    Logger.recordOutput("AimDebug/" + key + "/PivotVelY",         s.pivotFieldVelocity.getY());

    Logger.recordOutput("AimDebug/" + key + "/PivotPose",
        new Pose2d(s.pivotFieldPosition, new Rotation2d()));
    Logger.recordOutput("AimDebug/" + key + "/ReleasePose",
        new Pose2d(s.releasePivotFieldPosition, new Rotation2d()));
    Logger.recordOutput("AimDebug/" + key + "/AimPose",
        new Pose2d(s.releasePivotFieldPosition, Rotation2d.fromRadians(s.fieldAimAngleRad)));
    Logger.recordOutput("AimDebug/" + key + "/VirtualTarget",
        new Pose2d(s.aimPointField, new Rotation2d()));
  }

  // ── Result record ────────────────────────────────────────────────────────────

  private static final class AimSolution {
    final double        turretAngleDeg;
    final double        fieldAimAngleRad;
    final double        shotDistanceMeters;    // raw distance at release, no lead (logging only)
    final double        aimPathDistanceMeters; // lead-corrected distance (shot table + TOF input)
    final double        tofSeconds;
    final ShotSetpoint  shotSetpoint;
    final Translation2d pivotFieldPosition;        // pivot position right now
    final Translation2d releasePivotFieldPosition; // pivot position when ball exits
    final Translation2d aimPointField;             // virtual aim point after lead compensation
    final Translation2d pivotFieldVelocity;

    AimSolution(
        double        turretAngleDeg,
        double        fieldAimAngleRad,
        double        shotDistanceMeters,
        double        aimPathDistanceMeters,
        double        tofSeconds,
        ShotSetpoint  shotSetpoint,
        Translation2d pivotFieldPosition,
        Translation2d releasePivotFieldPosition,
        Translation2d aimPointField,
        Translation2d pivotFieldVelocity) {
      this.turretAngleDeg           = turretAngleDeg;
      this.fieldAimAngleRad         = fieldAimAngleRad;
      this.shotDistanceMeters       = shotDistanceMeters;
      this.aimPathDistanceMeters    = aimPathDistanceMeters;
      this.tofSeconds               = tofSeconds;
      this.shotSetpoint             = shotSetpoint;
      this.pivotFieldPosition       = pivotFieldPosition;
      this.releasePivotFieldPosition = releasePivotFieldPosition;
      this.aimPointField            = aimPointField;
      this.pivotFieldVelocity       = pivotFieldVelocity;
    }
  }
}