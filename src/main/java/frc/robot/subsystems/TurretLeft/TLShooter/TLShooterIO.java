package frc.robot.subsystems.turretleft.tlshooter;

import org.littletonrobotics.junction.AutoLog;

public interface TLShooterIO {
  @AutoLog
  public static class TLShooterIOInputs {
    public double supplyCurrent;
    public double motorEncoderValue;
    public double velocityRPM;
    public double tempCelcius;
  }

  public default void setSpeed(double speed) {}

  public default void hubAdaptiveAiming() {}

  public default void storageAdaptiveAiming() {}

  public default void updateInputs(TLShooterIOInputs inputs) {}
}
