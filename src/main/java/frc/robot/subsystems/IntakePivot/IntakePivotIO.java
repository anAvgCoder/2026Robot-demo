package frc.robot.subsystems.intakePivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  class IntakePivotIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempC = 0.0;
    public boolean magSensorTriggered = false;
    public double setpoint = 0.0;
    public boolean endstop;
  }

  void updateInputs(IntakePivotIOInputs inputs);

  void setVoltage(double volts);

  void seekPosition(double positionRad);

  /**
   * Attempt to home the intake
   *
   * @return Returns true when in home state
   */
  boolean seekHome();
}
