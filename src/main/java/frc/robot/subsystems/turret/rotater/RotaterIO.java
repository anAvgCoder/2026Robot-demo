package frc.robot.subsystems.turret.rotater;

import org.littletonrobotics.junction.AutoLog;

public interface RotaterIO {

  @AutoLog
  public static class RotaterIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempC = 0.0;
    public double goalPositionRad = 0.0;
    public double goalVelocityRadPerSec = 0.0;
    public boolean closedLoopActive = false;
    public boolean atGoal = false;
  }

  public default void setVoltage(double volts) {}

  public default void setTurnPosition(double degrees) {}

  public default void stop() {}

  public default boolean isAtGoal() {
    return false;
  }

  public default void updateInputs(RotaterIOInputs inputs) {}
}
