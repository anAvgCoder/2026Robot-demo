package frc.robot.subsystems.shooter;

public final class ShooterConstants {

  private ShooterConstants() {}

  // Hardware
  public static final int kCanIdLeft = 45;
  public static final int kCanIdRight = 44;
  public static final boolean kInverted = false;
  public static final int kCurrentLimitAmps = 60;
  public static final double kNominalVoltage = 12.0;

  // Feedforward
  public static final double kS = 0.15;
  public static final double kV = 0.00177;

  // PID
  public static final double kP = 0.00012;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  // Output limits
  public static final double kMinOutput = 0.0;
  public static final double kMaxOutput = 1.0;

  // At-goal detection
  public static final double kToleranceRPM = 75.0;
  public static final double kStableTimeSeconds = 0.15;
}
