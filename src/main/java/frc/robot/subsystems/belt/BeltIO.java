package frc.robot.subsystems.belt;

import org.littletonrobotics.junction.AutoLog;

public interface BeltIO {
  @AutoLog
  public static class BeltIOInputs {
    public boolean motorConnected = true;
    public double supplyCurrent;
    public double velocityRPM;
    public double tempCelcius;
    public double setpointRPM;
    public boolean paused;
  }

  public default void updateInputs(BeltIOInputs inputs) {}

  public default void intake() {}

  public default void outake() {}

  public default void stop() {}

  public default void setPaused(boolean value) {}
}
