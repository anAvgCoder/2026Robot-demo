package frc.robot.subsystems.diverter;

import org.littletonrobotics.junction.AutoLog;

public interface DiverterIO {

  @AutoLog
  public static class DiverterIOInputs {
    public boolean motorConnected = true;
    public double supplyCurrent;
    public double velocityRPM;
    public double tempCelcius;
    public boolean paused;
  }

  public default void updateInputs(DiverterIOInputs inputs) {}

  public default void intake() {}

  public default void outake() {}

  public default void stop() {}

  public void setPaused(boolean value);
}
