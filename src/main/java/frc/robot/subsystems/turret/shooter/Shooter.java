package frc.robot.subsystems.turret.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private enum ControlMode {
    DISABLED,
    CLOSED_LOOP_VELOCITY,
    OPEN_LOOP
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final String logKey;

  private ControlMode controlMode = ControlMode.DISABLED;
  private double desiredVelocityRPM = 0.0;
  private double desiredOpenLoopPercent = 0.0;

  public Shooter(ShooterIO io) {
    this(io, "Shooter");
  }

  public Shooter(ShooterIO io, String logKey) {
    this.io = io;
    this.logKey = logKey;
  }

  public ShooterIO getIO() {
    return io;
  }

  public void setVelocityRPM(double velocityRPM) {
    desiredVelocityRPM = Math.max(0.0, velocityRPM);
    controlMode = ControlMode.CLOSED_LOOP_VELOCITY;
  }

  public void setOpenLoopPercent(double percent) {
    desiredOpenLoopPercent = Math.max(-1.0, Math.min(1.0, percent));
    controlMode = ControlMode.OPEN_LOOP;
  }

  public void stop() {
    desiredVelocityRPM = 0.0;
    desiredOpenLoopPercent = 0.0;
    controlMode = ControlMode.DISABLED;
  }

  public boolean isAtSpeed() {
    return controlMode == ControlMode.CLOSED_LOOP_VELOCITY
        && Math.abs(inputs.goalVelocityRPM - inputs.velocityRPM) <= ShooterConstants.kVelocityToleranceRPM;
  }

  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }

  public double getGoalVelocityRPM() {
    return inputs.goalVelocityRPM;
  }

  public void setSpeed(double speed) {
    setVelocityRPM(speed);
  }

  public void setOpenSpeed(double speed) {
    setOpenLoopPercent(speed);
  }

  public void stopApplyingMotor() {
    stop();
  }

  public void startHubAdaptiveAiming() {
    setVelocityRPM(ShooterConstants.kDefaultHubAdaptiveRPM);
  }

  public void startStorageAdaptiveAiming() {
    setVelocityRPM(ShooterConstants.kDefaultStorageAdaptiveRPM);
  }

  @Override
  public void periodic() {
    switch (controlMode) {
      case CLOSED_LOOP_VELOCITY:
        io.setVelocityRPM(desiredVelocityRPM);
        break;
      case OPEN_LOOP:
        io.setOpenLoopPercent(desiredOpenLoopPercent);
        break;
      case DISABLED:
      default:
        io.stop();
        break;
    }

    io.updateInputs(inputs);
    Logger.processInputs(logKey, inputs);
    Logger.recordOutput(logKey + "/DesiredVelocityRPM", desiredVelocityRPM);
    Logger.recordOutput(logKey + "/DesiredOpenLoopPercent", desiredOpenLoopPercent);
    Logger.recordOutput(logKey + "/IsAtSpeed", isAtSpeed());
  }
}
