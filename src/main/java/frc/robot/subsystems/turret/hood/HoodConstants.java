package frc.robot.subsystems.turret.hood;

import edu.wpi.first.math.util.Units;

public class HoodConstants {
  // Hardware
  public static final int kRightCanId = 40; // TODO set
  public static final int kLeftCanId = 41; // TODO set
  public static final boolean kInverted = false;
  public static final int kCurrentLimitAmps = 15;

  // Hood range (degrees)
  public static final double kMinAnglePos = 0.0;
  public static final double kMaxAnglePos = 10.0;

  // Motion constraints (trapezoid)
  public static final double kMaxVelDegPerSec = 180.0;
  public static final double kMaxAccelDegPerSec2 = 360.0;

  // PID gains (TODO tune)
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  // Optional gravity/feedforward (TODO tune)
  public static final double kS = 0.0;
  public static final double kG = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;

  // Output limiting
  public static final double kMaxVolts = 12.0;

  // Tolerances
  public static final double kVelToleranceDegPerSec = 5.0;

  // --- Conversions (radians) ---

  public static final double kMaxVelRadPerSec = Units.degreesToRadians(kMaxVelDegPerSec);
  public static final double kMaxAccelRadPerSec2 = Units.degreesToRadians(kMaxAccelDegPerSec2);

  public static final double kPosToleranceRad = 0.03;
  public static final double kVelToleranceRadPerSec =
      Units.degreesToRadians(kVelToleranceDegPerSec);

  public static final double kGearRatio = 100.0; // TODO set

  // Motor rotations -> hood radians
  public static final double kPositionFactorRadPerMotorRot = (2.0 * Math.PI) / kGearRatio;

  // Motor RPM -> hood rad/s
  public static final double kVelocityFactorRadPerSecPerRPM = (2.0 * Math.PI) / (kGearRatio * 60.0);
}
