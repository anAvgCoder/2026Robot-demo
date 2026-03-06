package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.ShotTable;
import frc.robot.subsystems.turret.ShotTable.ShotSetpoint;
import frc.robot.subsystems.turret.shooter.Shooter;
import frc.robot.subsystems.turret.shooter.ShooterIO;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Stable hub aiming: - Uses current estimated robot pose for turret position on the field -
 * Computes turret angle to hub (robot-relative), applies mount offset - Sets hood + shooter from
 * ShotTable based on turret->hub distance
 *
 * <p>No shoot-on-the-move / lookahead here on purpose (stability first).
 */
public class ShooterAdaptiveHubAiming extends Command {
  private final ShooterIO shooterIORight;
  private final ShooterIO shooterIOLeft;

  private final Drive drive;
  private final boolean isBlue;

  private int runCounter;

  // IMPORTANT: these must be ROBOT_ORIGIN -> TURRET_PIVOT (NOT Quest origin).
  // Verify in CAD / tape measure for your chosen odometry origin.
  private static final double TURRET_X_INCHES = 6.5;
  private static final double TURRET_Y_INCHES = 6.75;

  public ShooterAdaptiveHubAiming(
      Shooter shooterRight, Shooter shooterLeft, Drive drive, boolean isBlueCheck) {

    this.shooterIORight = shooterRight.getIO();
    this.shooterIOLeft = shooterLeft.getIO();

    this.drive = drive;
    this.isBlue = isBlueCheck;

    addRequirements(shooterRight, shooterLeft);
  }

  @Override
  public void initialize() {
    runCounter = 0;
  }

  @Override
  public void execute() {
    Pose3d turretPoseRight = getTurretPose(true);

    double distRight = calculateHubDistance(turretPoseRight);
    ShotSetpoint spRight = ShotTable.get(distRight);
    shooterIORight.setSpeed(spRight.shooterSpeed());

    Pose3d turretPoseLeft = getTurretPose(false);

    double distLeft = calculateHubDistance(turretPoseLeft);
    ShotSetpoint spLeft = ShotTable.get(distLeft);
    shooterIOLeft.setSpeed(spLeft.shooterSpeed());

    runCounter++;
    if (runCounter > 24) {
      runCounter = 24;
    }
  }

  @Override
  public void end(boolean interrupted) {

    shooterIORight.setOpenSpeed(0.0);
    shooterIOLeft.setOpenSpeed(0.0);
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

  private Pose3d getTurretPose(boolean isRightTurret) {
    Pose3d robotPose = new Pose3d(drive.getPose());
    return robotPose.transformBy(robotToTurret(isRightTurret));
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

    // Debug output
    double aimFieldRad = robotYawRad + turretRelativeRad;
    Logger.recordOutput(
        "AimDebug/TurretAimPose",
        new Pose2d(turretPose.getX(), turretPose.getY(), new Rotation2d(aimFieldRad)));
    Logger.recordOutput(
        "AimDebug/TurretToHubPose",
        new Pose2d(turretPose.getX(), turretPose.getY(), new Rotation2d(fieldAngleRad)));

    return Math.toDegrees(turretRelativeRad);
  }

  public double calculateHubDistance(Pose3d turretPose) {
    Pose3d hubPose = isBlue ? FieldConstants.BLUE_HUB_POSE3D : FieldConstants.RED_HUB_POSE3D;
    double dx = turretPose.getX() - hubPose.getX();
    double dy = turretPose.getY() - hubPose.getY();
    return Math.hypot(dx, dy);
  }
}
