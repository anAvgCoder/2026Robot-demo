package frc.robot.subsystems.turret.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double position = 0.0;
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempC = 0.0;
  }

  public default void setVoltage(double volts) {}

  public default void zeroAtMin() {}

  public default void setHoodPosition(double degrees) {}

  public default void updateInputs(HoodIOInputs inputs) {}
}
