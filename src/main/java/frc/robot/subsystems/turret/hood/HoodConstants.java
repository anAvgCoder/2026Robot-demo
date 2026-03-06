package frc.robot.subsystems.turret.hood;

import edu.wpi.first.math.util.Units;

public class HoodConstants {
  // Hardware
  public static final int kRightCanId = 40;
  public static final int kLeftCanId = 41;
  public static final boolean kInverted = false;
  public static final int kCurrentLimitAmps = 15;

  // Hood range (radians)
  public static final double kMinAngleRad = 0.0;
  public static final double kMaxAngleRad = 0.439823;
  public static final double kStorageAngleRad = kMinAngleRad;

  // This hood uses only a relative encoder in the uploaded code. If true, boot assumes the hood is
  // mechanically already at kMinAngleRad. If false, an external homing/zeroing call is required.
  public static final boolean kAssumeMinAngleOnBoot = true;

  // PID gains. These remain in the same "duty-cycle-like" scale used by the original file so the
  // closed-loop behavior is preserved as closely as possible.
  public static final double kP = 0.65;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  // Manual output limiting
  public static final double kMaxManualVolts = 12.0;

  // Closed-loop duty-cycle limiting used with motor.set(...)
  public static final double kMaxClosedLoopDutyCycle = 1.0;

  // Motion constraints
  public static final double kMaxVelRadPerSec = 45.0;
  public static final double kMaxAccelRadPerSec2 = 70.0;

  // Tolerances
  public static final double kPosToleranceRad = 0.01;
  public static final double kVelToleranceDegPerSec = 5.0;
  public static final double kVelToleranceRadPerSec =
      Units.degreesToRadians(kVelToleranceDegPerSec);

  // Gear ratio: motor rotations per hood rotation
  public static final double kGearRatio = 100.0;

  // Motor rotations -> hood radians
  public static final double kPositionFactorRadPerMotorRot = (2.0 * Math.PI) / kGearRatio;

  // Motor RPM -> hood rad/s
  public static final double kVelocityFactorRadPerSecPerRPM =
      (2.0 * Math.PI) / (kGearRatio * 60.0);

  private HoodConstants() {}
}
