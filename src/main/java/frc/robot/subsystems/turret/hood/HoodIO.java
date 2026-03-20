package frc.robot.subsystems.turret.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempC = 0.0;
    public double goalPositionRad = 0.0;
    public double goalVelocityRadPerSec = 0.0;
    public boolean closedLoopActive = false;
    public boolean atGoal = false;
    public boolean zeroed = false;
    public boolean paused = false;
  }

  /** Sets manual voltage directly to the hood motor. */
  public default void setVoltage(double volts) {}

  /** Resets the hood encoder so the current position is treated as the minimum angle. */
  public default void zeroAtMin() {}

  /** Sets the hood target position in radians. */
  public default void setHoodPosition(double positionRad) {}

  /** Stops all hood output and exits closed loop. */
  public default void stop() {}

  public default boolean isAtGoal() {
    return false;
  }

  public default boolean isZeroed() {
    return false;
  }

  public default boolean getPaused() {
    return false;
  }

  public default void setPaused(boolean value) {}

  public default void updateInputs(HoodIOInputs inputs) {}
}
