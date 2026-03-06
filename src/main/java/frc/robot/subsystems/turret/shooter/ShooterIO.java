package frc.robot.subsystems.turret.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double motorPositionRot = 0.0;
    public double velocityRPM = 0.0;
    public double tempCelsius = 0.0;
    public double goalVelocityRPM = 0.0;
    public boolean closedLoopActive = false;
  }

  public default void setVelocityRPM(double velocityRPM) {}

  public default void setOpenLoopPercent(double percent) {}

  public default void stop() {}

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setSpeed(double speed) {
    setVelocityRPM(speed);
  }

  public default void setOpenSpeed(double speed) {
    setOpenLoopPercent(speed);
  }

  public default void stopApplyingMotor() {
    stop();
  }

  public default void hubAdaptiveAiming() {}

  public default void storageAdaptiveAiming() {}
}
