package frc.robot.subsystems.IntakeRoller;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {

  @AutoLog
  public static class IntakeRollerIOInputs {
    public boolean motorConnected = true;
    public double supplyCurrent;
    public double VelocityRPM;
    public double tempCelcius;
  }

  public default void updateInputs(IntakeRollerIOInputs inputs) {}

  public default void intake() {}

  public default void outtake() {}
}
