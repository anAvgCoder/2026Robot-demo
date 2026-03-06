package frc.robot.subsystems.turret.rotater;

import edu.wpi.first.math.util.Units;

public class RotaterConstants {
  // Turret Locations / mount zero offsets used by higher-level aim code.
  public static final double turretRightAngleLocation = 0.0;
  public static final double turretLeftAngleLocation = 0.0;

  // Hardware
  public static final int kCanIdLeft = 43;
  public static final int kCanIdRight = 42;
  public static final int kCanIdLeftCoder = 25;
  public static final int kCanIdRightCoder = 26;
  public static final boolean kInvertedRight = true;
  public static final boolean kInvertedLeft = true;
  public static final int kCurrentLimitAmps = 40;

  // Absolute encoder zero offsets.
  // Leave at 0.0 until you calibrate them to your real mechanism.
  public static final double kAbsEncoderOffsetRightRad = 0.0;
  public static final double kAbsEncoderOffsetLeftRad = 0.0;

  // Closed-loop motor command sign. Preserve the sign convention used in the original
  // implementation
  // until verified on-robot.
  public static final double kClosedLoopOutputSignRight = -1.0;
  public static final double kClosedLoopOutputSignLeft = -1.0;

  // Soft limits (degrees)
  public static final double kMinAngleDeg = -135.0;
  public static final double kMaxAngleDeg = 135.0;

  // Motion constraints for the turret profile controller
  public static final double kMaxVelRadPerSec = 45.0;
  public static final double kMaxAccelRadPerSec2 = 75.0;

  // Gains TODO: TUNE
  // Units here are effectively volts-per-radian, etc., because the closed-loop output is applied
  // with setVoltage().
  public static final double kP = 0.5;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  // Output limiting
  public static final double kMaxVolts = 8.0;

  // Tolerances
  public static final double kPosToleranceDeg = 1.0;
  public static final double kVelToleranceDegPerSec = 5.0;

  public static final double kMinAngleRad = Units.degreesToRadians(kMinAngleDeg);
  public static final double kMaxAngleRad = Units.degreesToRadians(kMaxAngleDeg);
  public static final double kPosToleranceRad = Units.degreesToRadians(kPosToleranceDeg);
  public static final double kVelToleranceRadPerSec =
      Units.degreesToRadians(kVelToleranceDegPerSec);

  private RotaterConstants() {}
}
