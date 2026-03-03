package frc.robot.commands.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.questnav.QuestNavSystemIO;
import frc.robot.subsystems.turret.ShotTimeTable;
import frc.robot.subsystems.turret.rotater.Rotater;
import frc.robot.subsystems.turret.rotater.RotaterConstants;
import frc.robot.subsystems.turret.rotater.RotaterIO;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

// new AdaptiveHubAiming(rotater, shooter, hood,
// questNavSystem).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
public class AdaptiveHubAimingOnlyTurret extends Command {
  private final RotaterIO rotaterIORight;
  private final RotaterIO rotaterIOLeft;

  private final QuestNavSystemIO questNavSystemIO;
  private boolean isBlue = true;
  private int runCounter;

  public AdaptiveHubAimingOnlyTurret(
      Rotater rotaterRight, Rotater rotaterLeft, Drive drive, boolean isBlueCheck) {
    rotaterIORight = rotaterRight.getIO();
    rotaterIOLeft = rotaterLeft.getIO();

    questNavSystemIO = drive.getQuestNavSystemIO();

    addRequirements(rotaterRight, rotaterLeft);
    isBlue = isBlueCheck;
  }

  @Override
  public void initialize() {
    runCounter = 0;
  }

  @Override
  public void execute() {
    // Pose3d turretPoseRight = calculateAdjustedTurretPose(true);
    // rotaterIORight.setTurnPosition(
    //     calculateTurretDegreesRobotRelative(
    //         turretPoseRight, RotaterConstants.turretRightAngleLocation));

    Pose3d turretPoseLeft = calculateAdjustedTurretPose(false);
    rotaterIOLeft.setTurnPosition(
        calculateTurretDegreesRobotRelative(
            turretPoseLeft, RotaterConstants.turretLeftAngleLocation));

    runCounter++;
    if (runCounter > 24) {
      runCounter = 0;
    }
  }

  @Override
  public void end(boolean interrupted) {}

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
    Pose3d hubPose = isBlue ? FieldConstants.BLUE_HUB_POSE3D : FieldConstants.RED_HUB_POSE3D;

    double dx = hubPose.getX() - turretPose.getX();
    double dy = hubPose.getY() - turretPose.getY();

    double fieldAngleRad = Math.atan2(dy, dx);
    double robotYawRad = turretPose.getRotation().getZ();
    // double turretMountRad = Math.toRadians(turretMountAngleDeg);
    double turretRelativeRad = MathUtil.angleModulus(fieldAngleRad - robotYawRad);

    double aimFieldRad = robotYawRad + turretRelativeRad;
    Logger.recordOutput(
        "AimDebug/TurretAimPose",
        new Pose2d(turretPose.getX(), turretPose.getY(), new Rotation2d(aimFieldRad)));

    // Log where it SHOULD be pointing (straight at hub)
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

  public Pose3d calculateAdjustedTurretPose(boolean isRightTurret) {
    Pose3d robotPose;

    if (isRightTurret) {
      robotPose =
          questNavSystemIO.predictPoseFromWindow(
              questNavSystemIO.getLast6RobotPoses(),
              ShotTimeTable.getFlightTimeSeconds(calculateTurretDistance(true)));
    } else {
      robotPose =
          questNavSystemIO.predictPoseFromWindow(
              questNavSystemIO.getLast6RobotPoses(),
              ShotTimeTable.getFlightTimeSeconds(calculateTurretDistance(false)));
    }

    if (isRightTurret) {
      robotPose =
          robotPose.transformBy(pointOnCircleDegCCW(RotaterConstants.turretRightAngleLocation));
    } else {
      robotPose =
          robotPose.transformBy(pointOnCircleDegCCW(RotaterConstants.turretLeftAngleLocation));
    }

    return robotPose;
  }

  public double calculateTurretDistance(boolean isRightTurret) {
    Pose3d robotPose = questNavSystemIO.getLastRobotPose();

    if (isRightTurret) {
      robotPose =
          robotPose.transformBy(pointOnCircleDegCCW(RotaterConstants.turretRightAngleLocation));
    } else {
      robotPose =
          robotPose.transformBy(pointOnCircleDegCCW(RotaterConstants.turretLeftAngleLocation));
    }

    double dx;
    double dy;

    if (isBlue) {
      dx = robotPose.getX() - FieldConstants.BLUE_HUB_POSE3D.getX();
      dy = robotPose.getY() - FieldConstants.BLUE_HUB_POSE3D.getY();
    } else {
      dx = robotPose.getX() - FieldConstants.RED_HUB_POSE3D.getX();
      dy = robotPose.getY() - FieldConstants.RED_HUB_POSE3D.getY();
    }
    return Math.hypot(dx, dy);
  }

  public Transform3d pointOnCircleDegCCW(double angleDeg) {
    double r = Units.inchesToMeters(9.37); // radius of the circle 6.5^2 + 6.75^2
    double theta = Math.toRadians(angleDeg);

    double x = -r * Math.sin(theta);
    double y = r * Math.cos(theta);

    return new Transform3d(x, y, 0.0, new Rotation3d(0.0, 0.0, 0.0));
  }
}
