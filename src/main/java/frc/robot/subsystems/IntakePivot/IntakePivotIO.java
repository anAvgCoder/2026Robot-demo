package frc.robot.subsystems.intakepivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  public static class IntakePivotIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempC = 0.0;
  }

  public default void setVoltage(double volts) {}

  public default void setStoragePosition() {}

  public default void setIntakePosition() {}

  public default void setPivotPosition(double degrees) {}

  public default void zeroToStorage() {}

  public default void zeroToIntake() {}

  public default void updateInputs(IntakePivotIOInputs inputs) {}
}
