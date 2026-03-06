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
import frc.robot.subsystems.turret.hood.HoodIO;
import frc.robot.subsystems.turret.rotater.Rotater;
import frc.robot.subsystems.turret.rotater.RotaterIO;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class AdaptiveHubNoMove extends Command {
  private final RotaterIO rotaterIORight;
  private final HoodIO hoodIORight;
  private final RotaterIO rotaterIOLeft;
  private final HoodIO hoodIOLeft;

  private final Drive drive;
  private final boolean isBlue;

  private int runCounter;

  public AdaptiveHubNoMove(
      Rotater rotaterRight,
      Hood hoodRight,
      Rotater rotaterLeft,
      Hood hoodLeft,
      Drive drive,
      boolean isBlueCheck) {

    rotaterIORight = rotaterRight.getIO();
    hoodIORight = hoodRight.getIO();

    rotaterIOLeft = rotaterLeft.getIO();
    hoodIOLeft = hoodLeft.getIO();

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
    Pose3d turretPoseRight = getTurretPose(true);
    rotaterIORight.setTurnPosition(calculateTurretDegreesRobotRelative(turretPoseRight));
    ShotSetpoint spR = ShotTable.get(calculateAdjustedHubDistance(turretPoseRight));
    hoodIORight.setHoodPosition(spR.hoodPos());

    Pose3d turretPoseLeft = getTurretPose(false);
    rotaterIOLeft.setTurnPosition(calculateTurretDegreesRobotRelative(turretPoseLeft));
    ShotSetpoint spL = ShotTable.get(calculateAdjustedHubDistance(turretPoseLeft));
    hoodIOLeft.setHoodPosition(spL.hoodPos());

    runCounter++;
    if (runCounter > 24) runCounter = 0;
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

  private Transform3d turretOffset(boolean isRightTurret) {
    double xMeters = Units.inchesToMeters(6.5);
    double yMeters = Units.inchesToMeters(isRightTurret ? -6.75 : 6.75);
    return new Transform3d(xMeters, yMeters, 0.0, new Rotation3d());
  }

  private Pose3d getTurretPose(boolean isRightTurret) {
    Pose3d robotPose = new Pose3d(drive.getPose());
    return robotPose.transformBy(turretOffset(isRightTurret));
  }

  public double calculateTurretDegreesRobotRelative(Pose3d turretPose) {
    Pose3d targetPose = isBlue ? FieldConstants.BLUE_HUB_POSE3D : FieldConstants.RED_HUB_POSE3D;

    double dx = targetPose.getX() - turretPose.getX();
    double dy = targetPose.getY() - turretPose.getY();

    double fieldAngleRad = Math.atan2(dy, dx);
    double robotYawRad = turretPose.getRotation().getZ();

    double turretRelativeRad = MathUtil.angleModulus(fieldAngleRad - robotYawRad);

    double aimFieldRad = robotYawRad;
    Logger.recordOutput(
        "AimNoMove/TurretAimPose",
        new Pose2d(turretPose.getX(), turretPose.getY(), new Rotation2d(aimFieldRad)));
    Logger.recordOutput(
        "AimNoMove/TurretToTargetPose",
        new Pose2d(turretPose.getX(), turretPose.getY(), new Rotation2d(fieldAngleRad)));

    return Math.toDegrees(turretRelativeRad);
  }

  public double calculateAdjustedHubDistance(Pose3d turretPose) {
    double hubX =
        isBlue ? FieldConstants.BLUE_HUB_POSE3D.getX() : FieldConstants.RED_HUB_POSE3D.getX();
    double hubY =
        isBlue ? FieldConstants.BLUE_HUB_POSE3D.getY() : FieldConstants.RED_HUB_POSE3D.getY();

    double dx = turretPose.getX() - hubX;
    double dy = turretPose.getY() - hubY;
    return Math.hypot(dx, dy);
  }
}
