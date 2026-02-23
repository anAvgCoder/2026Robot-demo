package frc.robot.subsystems.turret.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double supplyCurrent;
    public double motorEncoderValue;
    public double velocityRPM;
    public double tempCelcius;
  }

  public default void setSpeed(double speed) {}

  public default void hubAdaptiveAiming() {}

  public default void storageAdaptiveAiming() {}

  public default void updateInputs(ShooterIOInputs inputs) {}
}
