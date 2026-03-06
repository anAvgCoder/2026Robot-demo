package frc.robot.commands.turret;

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
import frc.robot.subsystems.turret.hood.Hood;
import frc.robot.subsystems.turret.rotater.Rotater;
import frc.robot.subsystems.turret.rotater.RotaterConstants;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class AdaptiveNoMove extends Command {
  private final Rotater rotaterRight;
  private final Hood hoodRight;
  private final Rotater rotaterLeft;
  private final Hood hoodLeft;
  private final Drive drive;
  private final boolean isBlue;

  public AdaptiveNoMove(
      Rotater rotaterRight,
      Hood hoodRight,
      Rotater rotaterLeft,
      Hood hoodLeft,
      Drive drive,
      boolean isBlueCheck) {
    this.rotaterRight = rotaterRight;
    this.hoodRight = hoodRight;
    this.rotaterLeft = rotaterLeft;
    this.hoodLeft = hoodLeft;
    this.drive = drive;
    this.isBlue = isBlueCheck;

    addRequirements(rotaterRight, hoodRight, rotaterLeft, hoodLeft);
  }

  @Override
  public void execute() {
    Pose3d turretPoseRight = getTurretPose(true);
    rotaterRight.setTurnPosition(
        calculateTurretDegreesRobotRelative(
            turretPoseRight, RotaterConstants.turretRightAngleLocation));
    ShotSetpoint spRight = ShotTable.get(calculateAdjustedTargetDistance(turretPoseRight));
    hoodRight.setHoodPosition(spRight.hoodPos());

    Pose3d turretPoseLeft = getTurretPose(false);
    rotaterLeft.setTurnPosition(
        calculateTurretDegreesRobotRelative(
            turretPoseLeft, RotaterConstants.turretLeftAngleLocation));
    ShotSetpoint spLeft = ShotTable.get(calculateAdjustedTargetDistance(turretPoseLeft));
    hoodLeft.setHoodPosition(spLeft.hoodPos());
  }

  @Override
  public void end(boolean interrupted) {
    rotaterRight.stop();
    rotaterLeft.stop();
    hoodRight.stop();
    hoodLeft.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private Transform3d turretOffset(boolean isRightTurret) {
    double xMeters = Units.inchesToMeters(6.5);
    double yMeters = Units.inchesToMeters(isRightTurret ? -6.75 : 6.75);
    return new Transform3d(xMeters, yMeters, 0.0, new Rotation3d());
  }

  private Pose3d getTurretPose(boolean isRightTurret) {
    Pose3d robotPose = new Pose3d(drive.getPose());
    return robotPose.transformBy(turretOffset(isRightTurret));
  }

  public double calculateTurretDegreesRobotRelative(Pose3d turretPose, double turretMountAngleDeg) {
    Pose3d targetPose =
        isBlue ? FieldConstants.BLUE_LOW_TARGET_POSE3D : FieldConstants.RED_HIGH_TARGET_POSE3D;

    double dx = targetPose.getX() - turretPose.getX();
    double dy = targetPose.getY() - turretPose.getY();

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
        "AimDebug/TurretToTargetPose",
        new Pose2d(turretPose.getX(), turretPose.getY(), new Rotation2d(fieldAngleRad)));

    return Math.toDegrees(turretRelativeRad);
  }

  public double calculateAdjustedTargetDistance(Pose3d turretPose) {
    Pose3d targetPose =
        isBlue ? FieldConstants.BLUE_LOW_TARGET_POSE3D : FieldConstants.RED_HIGH_TARGET_POSE3D;

    double dx = turretPose.getX() - targetPose.getX();
    double dy = turretPose.getY() - targetPose.getY();
    return Math.hypot(dx, dy);
  }
}
