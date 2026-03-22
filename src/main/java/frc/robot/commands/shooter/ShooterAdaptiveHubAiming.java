package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.ShotTable;
import frc.robot.subsystems.turret.ShotTable.ShotSetpoint;
import frc.robot.subsystems.turret.shooter.Shooter;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class ShooterAdaptiveHubAiming extends Command {
  private final Shooter shooterRight;
  private final Shooter shooterLeft;
  private final Drive drive;
  private final boolean isBlue;
  private final ShotTable shotTable;

  private static final double TURRET_X_INCHES = 6.5;
  private static final double TURRET_Y_INCHES = 6.75;

  public ShooterAdaptiveHubAiming(
      Shooter shooterRight,
      Shooter shooterLeft,
      Drive drive,
      boolean isBlueCheck,
      ShotTable shotTable) {
    this.shooterRight = shooterRight;
    this.shooterLeft = shooterLeft;
    this.drive = drive;
    this.isBlue = isBlueCheck;
    this.shotTable = shotTable;

    addRequirements(shooterRight, shooterLeft);
  }

  @Override
  public void execute() {
    Pose3d turretPoseRight = getTurretPose(true);
    double distRight = calculateHubDistance(turretPoseRight);
    ShotSetpoint shotRight = shotTable.get(distRight);
    shooterRight.setVelocityRPM(shotRight.shooterSpeed());

    Logger.recordOutput("AimDebug/" + "Right" + "/ShooterDist", distRight);
    Logger.recordOutput("AimDebug/" + "Right" + "/ShooterTargetRPM", shotRight.shooterSpeed());

    Pose3d turretPoseLeft = getTurretPose(false);
    double distLeft = calculateHubDistance(turretPoseLeft);
    ShotSetpoint shotLeft = shotTable.get(distLeft);
    shooterLeft.setVelocityRPM(shotLeft.shooterSpeed());

    Logger.recordOutput("AimDebug/" + "Left" + "/ShooterDist", distLeft);
    Logger.recordOutput("AimDebug/" + "Left" + "/ShooterTargetRPM", shotLeft.shooterSpeed());
  }

  @Override
  public void end(boolean interrupted) {
    shooterRight.stop();
    shooterLeft.stop();
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

  public double calculateHubDistance(Pose3d turretPose) {
    Pose3d hubPose = isBlue ? FieldConstants.BLUE_HUB_POSE3D : FieldConstants.RED_HUB_POSE3D;
    double dx = turretPose.getX() - hubPose.getX();
    double dy = turretPose.getY() - hubPose.getY();
    return Math.hypot(dx, dy);
  }
}
