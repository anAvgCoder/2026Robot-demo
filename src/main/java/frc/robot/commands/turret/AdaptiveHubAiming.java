package frc.robot.commands.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
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

// new AdaptiveHubAiming(rotater, shooter, hood,
// questNavSystem).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
public class AdaptiveHubAiming extends Command {
  private final RotaterIO rotaterIORight;
  private final HoodIO hoodIORight;
  private final ShooterIO shooterIORight;
  private final RotaterIO rotaterIOLeft;
  private final HoodIO hoodIOLeft;
  private final ShooterIO shooterIOLeft;
  private final QuestNavSystemIO questNavSystemIO;
  private boolean isBlue = true;
  private int runCounter;

  public AdaptiveHubAiming(
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
    ShotSetpoint shotSetpointRight = ShotTable.get(calculateAdjustedHubDistance(turretPoseRight));
    hoodIORight.setHoodPosition(shotSetpointRight.hoodPos());
    shooterIORight.setSpeed(shotSetpointRight.shooterSpeed());
    rotaterIORight.setTurnPosition(calculateTurretDegreesRobotRelative(turretPoseRight));

    Pose3d turretPoseLeft = calculateAdjustedTurretPose(false);
    ShotSetpoint shotSetpointLeft = ShotTable.get(calculateAdjustedHubDistance(turretPoseLeft));
    hoodIOLeft.setHoodPosition(shotSetpointLeft.hoodPos());
    shooterIOLeft.setSpeed(shotSetpointLeft.shooterSpeed());
    rotaterIOLeft.setTurnPosition(calculateTurretDegreesRobotRelative(turretPoseLeft));

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

  public double calculateTurretDegreesRobotRelative(Pose3d turretPose) {
    Pose3d hubPose = isBlue ? FieldConstants.BLUE_HUB_POSE3D : FieldConstants.RED_HUB_POSE3D;

    double dx = hubPose.getX() - turretPose.getX();
    double dy = hubPose.getY() - turretPose.getY();

    // Field-relative angle to hub
    double fieldAngleRad = Math.atan2(dy, dx);
    // Robot's yaw
    double robotYawRad = turretPose.getRotation().getZ();
    // Subtract robot yaw to get robot-relative angle, then wrap to ±180°
    double robotRelativeRad = MathUtil.angleModulus(fieldAngleRad - robotYawRad);

    return Math.toDegrees(robotRelativeRad);
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
