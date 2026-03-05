package frc.robot.commands.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.questnav.QuestNavSystemConstants;
import frc.robot.subsystems.questnav.QuestNavSystemIO;
import frc.robot.subsystems.turret.ShotTable;
import frc.robot.subsystems.turret.ShotTable.ShotSetpoint;
import frc.robot.subsystems.turret.ShotTimeTable;
import frc.robot.subsystems.turret.hood.Hood;
import frc.robot.subsystems.turret.hood.HoodIO;
import frc.robot.subsystems.turret.rotater.Rotater;
import frc.robot.subsystems.turret.rotater.RotaterConstants;
import frc.robot.subsystems.turret.rotater.RotaterIO;
import frc.robot.subsystems.turret.shooter.Shooter;
import frc.robot.subsystems.turret.shooter.ShooterIO;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

// new AdaptiveHubAiming(rotater, shooter, hood,
// questNavSystem).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
public class AdaptiveStorageAiming extends Command {
  private final RotaterIO rotaterIORight;
  private final HoodIO hoodIORight;
  private final ShooterIO shooterIORight;
  private final RotaterIO rotaterIOLeft;
  private final HoodIO hoodIOLeft;
  private final ShooterIO shooterIOLeft;
  private Drive drive;
  private final QuestNavSystemIO questNavSystemIO;
  private boolean isBlue = true;
  private int runCounter;

  public AdaptiveStorageAiming(
      Rotater rotaterRight,
      Shooter shooterRight,
      Hood hoodRight,
      Rotater rotaterLeft,
      Shooter shooterLeft,
      Hood hoodLeft,
      Drive drive,
      boolean isBlueCheck) {
    rotaterIORight = rotaterRight.getIO();
    hoodIORight = hoodRight.getIO();
    shooterIORight = shooterRight.getIO();

    rotaterIOLeft = rotaterLeft.getIO();
    hoodIOLeft = hoodLeft.getIO();
    shooterIOLeft = shooterLeft.getIO();

    this.drive = drive;

    questNavSystemIO = drive.getQuestNavSystemIO();

    addRequirements(rotaterRight, shooterRight, hoodRight, rotaterLeft, shooterLeft, hoodLeft);

    isBlue = isBlueCheck;
  }

  @Override
  public void initialize() {
    runCounter = 0;
  }

  @Override
  public void execute() {

    Pose3d turretPoseRight = calculateAdjustedTurretPose(true);
    rotaterIORight.setTurnPosition(
        calculateTurretDegreesRobotRelative(
            turretPoseRight, RotaterConstants.turretRightAngleLocation));
    ShotSetpoint shotSetpointRight = ShotTable.get(calculateAdjustedHubDistance(turretPoseRight));
    hoodIORight.setHoodPosition(shotSetpointRight.hoodPos());
    shooterIORight.setSpeed(shotSetpointRight.shooterSpeed());

    Pose3d turretPoseLeft = calculateAdjustedTurretPose(false);
    rotaterIOLeft.setTurnPosition(
        calculateTurretDegreesRobotRelative(
            turretPoseLeft, RotaterConstants.turretLeftAngleLocation));
    ShotSetpoint shotSetpointLeft = ShotTable.get(calculateAdjustedHubDistance(turretPoseLeft));
    hoodIOLeft.setHoodPosition(shotSetpointLeft.hoodPos());
    shooterIOLeft.setSpeed(shotSetpointLeft.shooterSpeed());

    runCounter++;
    if (runCounter > 24) {
      runCounter = 0;
    }
  }

  @Override
  public void end(boolean interrupted) {
    rotaterIORight.setVoltage(0.0);
    rotaterIOLeft.setVoltage(0.0);
    shooterIORight.setOpenSpeed(0.0);
    shooterIOLeft.setOpenSpeed(0.0);
    hoodIORight.setVoltage(0.0);
    hoodIOLeft.setVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    if (!DriverStation.isAutonomous()) {
      return false;
    } else {
      if (runCounter == 24) {
        return true;
      } else {
        return false;
      }
    }
  }

  public double calculateTurretDegreesRobotRelative(Pose3d turretPose, double turretMountAngleDeg) {
    Pose3d hubPose =
        isBlue ? FieldConstants.BLUE_LOW_TARGET_POSE3D : FieldConstants.RED_HIGH_TARGET_POSE3D;

    double dx = hubPose.getX() - turretPose.getX();
    double dy = hubPose.getY() - turretPose.getY();

    double fieldAngleRad = Math.atan2(dy, dx);
    double robotYawRad = turretPose.getRotation().getZ();
    double turretRelativeRad = MathUtil.angleModulus(fieldAngleRad - robotYawRad);

    double aimFieldRad = robotYawRad + turretRelativeRad;
    Logger.recordOutput(
        "AimDebug/TurretAimPose",
        new Pose2d(turretPose.getX(), turretPose.getY(), new Rotation2d(aimFieldRad)));

    Logger.recordOutput(
        "AimDebug/TurretToHubPose",
        new Pose2d(turretPose.getX(), turretPose.getY(), new Rotation2d(fieldAngleRad)));

    return Math.toDegrees(turretRelativeRad);
  }

  public double calculateAdjustedHubDistance(Pose3d turretPose) {

    double dx;
    double dy;

    if (isBlue) {
      dx = turretPose.getX() - FieldConstants.BLUE_HUB_POSE3D.getX();
      dy = turretPose.getY() - FieldConstants.BLUE_HUB_POSE3D.getY();
    } else {
      dx = turretPose.getX() - FieldConstants.RED_HUB_POSE3D.getX();
      dy = turretPose.getY() - FieldConstants.RED_HUB_POSE3D.getY();
    }

    return Math.hypot(dx, dy);
  }

  private static final int LOOKAHEAD_ITERS = 3;

  private Pose3d predictPoseFromFieldSpeeds(Pose3d start, ChassisSpeeds field, double dtSec) {
    double newX = start.getX() + field.vxMetersPerSecond * dtSec;
    double newY = start.getY() + field.vyMetersPerSecond * dtSec;
    double newYaw = start.getRotation().getZ() + field.omegaRadiansPerSecond * dtSec;
    return new Pose3d(newX, newY, start.getZ(), new Rotation3d(0.0, 0.0, newYaw));
  }

  private Transform3d turretOffset(boolean isRightTurret) {
    double xMeters =
        Units.inchesToMeters(6.5 + QuestNavSystemConstants.ROBOT_TO_QUEST_INCHES_X_DOUBLE);
    double yMeters = Units.inchesToMeters(isRightTurret ? -6.75 : 6.75);
    return new Transform3d(xMeters, yMeters, 0.0, new Rotation3d());
  }

  public Pose3d calculateAdjustedTurretPose(boolean isRightTurret) {
    Pose3d baseRobotPose = new Pose3d(drive.getPose());

    ChassisSpeeds fieldSpeeds = drive.getFieldRelativeSpeeds();

    Pose3d predictedRobotPose = baseRobotPose;
    Pose3d predictedTurretPose =
        new Pose3d(
            predictedRobotPose
                .getTranslation()
                .plus(
                    turretOffset(isRightTurret)
                        .getTranslation()
                        .rotateBy(predictedRobotPose.getRotation())),
            predictedRobotPose.getRotation());

    double dist = calculateAdjustedHubDistance(predictedTurretPose);
    double tof = ShotTimeTable.getFlightTimeSeconds(dist);

    for (int i = 0; i < LOOKAHEAD_ITERS; i++) {
      predictedRobotPose =
          predictPoseFromFieldSpeeds(new Pose3d(drive.getPose()), fieldSpeeds, tof);
      predictedTurretPose =
          new Pose3d(
              predictedRobotPose
                  .getTranslation()
                  .plus(
                      turretOffset(isRightTurret)
                          .getTranslation()
                          .rotateBy(predictedRobotPose.getRotation())),
              predictedRobotPose.getRotation());

      dist = calculateAdjustedHubDistance(predictedTurretPose);
      tof = ShotTimeTable.getFlightTimeSeconds(dist);
    }

    return predictedTurretPose;
  }
}
