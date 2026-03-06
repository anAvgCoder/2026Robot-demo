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
    public boolean magSensorTriggered = false;
    public boolean hasBeenZeroed = false;
    public double goalPositionRad = 0.0;
  }

  public default void setVoltage(double volts) {}

  public default void setStoragePosition() {}

  public default void setIntakePrimaryPosition() {}

  public default void setIntakeSecondaryPosition() {}

  public default void setPivotPosition(double positionRad) {}

  public default boolean isAtGoal() {
    return false;
  }

  public default void zeroToStorage() {}

  public default void zeroToIntake() {}

  public default void updateInputs(IntakePivotIOInputs inputs) {}
}
