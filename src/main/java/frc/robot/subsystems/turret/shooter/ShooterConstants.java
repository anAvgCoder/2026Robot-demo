package frc.robot.subsystems.turret.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
  // Spark Flex + NEO Vortex
  public static final int CanIdRight = 44;
  public static final int CanIdLeft = 45;

  public static final boolean kInverted = true;
  public static final int kCurrentLimitAmps = 40;
  public static final double kNominalVoltage = 12.0;

  // Closed-loop gains used by the Spark controller in velocity mode (RPM-native units).
  public static final double kP = 0.00006;
  public static final double kV = 0.00184;

  // Subsystem behavior / diagnostics.
  public static final double kVelocityToleranceRPM = 100.0;
  public static final double kDefaultHubAdaptiveRPM = 0.0;
  public static final double kDefaultStorageAdaptiveRPM = 0.0;

  // Simulation constants.
  public static final double kLoopPeriodSec = 0.02;
  public static final double kSimJkgMetersSquared = 0.025;
  public static final double kSimGearing = 1.0;
  public static final double kSimVelocityKpVoltsPerRPM = 0.003;
  public static final DCMotor kSimMotor = DCMotor.getNeoVortex(1);
  public static final double kNeoVortexFreeSpeedRPM =
      Units.radiansPerSecondToRotationsPerMinute(kSimMotor.freeSpeedRadPerSec);

  private ShooterConstants() {}
}
