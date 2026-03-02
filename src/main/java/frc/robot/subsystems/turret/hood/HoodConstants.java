package frc.robot.subsystems.turret.hood;

import edu.wpi.first.math.util.Units;

public class HoodConstants {
  // Hardware
  public static final int kRightCanId = 40; // TODO set
  public static final int kLeftCanId = 41; // TODO set
  public static final boolean kInverted = false;
  public static final int kCurrentLimitAmps = 15;

  // Hood range (radians)
  public static final double kMinAngleRad = 0;
  public static final double kMaxAngleRad = -1.47;

  // PID gains (TODO tune)
  public static final double kP = 0.65;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  // Output limiting
  public static final double kMaxVolts = 4.0;
  public static final double kMaxOutput = 1.0; // duty cycle cap for motor.set()

  // Tolerances
  public static final double kVelToleranceDegPerSec = 5.0;

  // --- Conversions (radians) ---

  public static final double kMaxVelRadPerSec = 45;
  public static final double kMaxAccelRadPerSec2 = 70;

  public static final double kPosToleranceRad = 0.03;
  public static final double kVelToleranceRadPerSec =
      Units.degreesToRadians(kVelToleranceDegPerSec);

  public static final double kGearRatio = 100.0; // TODO set

  // Motor rotations -> hood radians
  public static final double kPositionFactorRadPerMotorRot = (2.0 * Math.PI) / kGearRatio;

  // Motor RPM -> hood rad/s
  public static final double kVelocityFactorRadPerSecPerRPM = (2.0 * Math.PI) / (kGearRatio * 60.0);
}
