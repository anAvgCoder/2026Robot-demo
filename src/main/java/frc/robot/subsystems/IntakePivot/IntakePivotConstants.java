package frc.robot.subsystems.intakepivot;

import edu.wpi.first.math.util.Units;

public class IntakePivotConstants {
  // Hardware
  public static final int kCanId = 50; // TODO
  public static final boolean kInverted = false;
  public static final int kCurrentLimitAmps = 30;

  // Mechanism conversion
  public static final double kGearRatio = 100.0; // TODO

  // Positions
  public static final double kStoragePosition = 0.0; // TODO
  public static final double kIntakePosition = 30.0; // TODO

  // Trapezoid constraints
  public static final double kMaxVelDegPerSec = 180.0;
  public static final double kMaxAccelDegPerSec2 = 360.0;

  // PID gains (TODO tune)
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  // Optional feedforward (set to 0.0 if you’re not using it)
  public static final double kS = 0.0;
  public static final double kG = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;

  // Output limiting
  public static final double kMaxVolts = 12.0;

  // Tolerances
  public static final double kPosToleranceDeg = 1.0;
  public static final double kVelToleranceDegPerSec = 5.0;

  public static final double kMaxVelRadPerSec = Units.degreesToRadians(kMaxVelDegPerSec);
  public static final double kMaxAccelRadPerSec2 = Units.degreesToRadians(kMaxAccelDegPerSec2);

  public static final double kPosToleranceRad = Units.degreesToRadians(kPosToleranceDeg);
  public static final double kVelToleranceRadPerSec =
      Units.degreesToRadians(kVelToleranceDegPerSec);

  public static final double kPositionFactorRadPerMotorRot = (2.0 * Math.PI) / kGearRatio;

  public static final double kVelocityFactorRadPerSecPerRPM = (2.0 * Math.PI) / (kGearRatio * 60.0);
}
