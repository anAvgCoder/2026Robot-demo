package frc.robot.subsystems.turret.rotater;

import edu.wpi.first.math.util.Units;

public class RotaterConstants {
  // Turret Locations
  public static final double turretRightAngleLocation = 46; // TODO: measure with calc
  public static final double turretLeftAngleLocation = -44;

  // Hardware
  public static final int kCanIdLeft = 43;
  public static final int kCanIdRight = 42;
  public static final int kCanIdLeftCoder = 25;
  public static final int kCanIdRightCoder = 26;
  public static final boolean kInverted = false;
  public static final int kCurrentLimitAmps = 30;

  // Motor rotations per turret rotation
  public static final double kGearRatio = 100.0;

  // Soft limits (degrees)
  public static final double kMinAngleDeg = -180.0;
  public static final double kMaxAngleDeg = 180.0;

  // Motion constraints for the trapezoidal pid thing from wpilib
  public static final double kMaxVelDegPerSec = 240.0;
  public static final double kMaxAccelDegPerSec2 = 720.0;

  // Gains TODO: TUNE
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  // Output limiting
  public static final double kMaxVolts = 12.0;

  // Tolerances
  public static final double kPosToleranceDeg = 1.0;
  public static final double kVelToleranceDegPerSec = 5.0;

  // --- Conversions (radians) ---
  public static final double kMinAngleRad = Units.degreesToRadians(kMinAngleDeg);
  public static final double kMaxAngleRad = Units.degreesToRadians(kMaxAngleDeg);

  public static final double kMaxVelRadPerSec = Units.degreesToRadians(kMaxVelDegPerSec);
  public static final double kMaxAccelRadPerSec2 = Units.degreesToRadians(kMaxAccelDegPerSec2);

  public static final double kPosToleranceRad = Units.degreesToRadians(kPosToleranceDeg);
  public static final double kVelToleranceRadPerSec =
      Units.degreesToRadians(kVelToleranceDegPerSec);

  // Motor rotations -> turret radians
  public static final double kPositionFactorRadPerMotorRot = (2.0 * Math.PI) / kGearRatio;

  // Motor RPM -> turret rad/s
  public static final double kVelocityFactorRadPerSecPerRPM = (2.0 * Math.PI) / (kGearRatio * 60.0);
}
