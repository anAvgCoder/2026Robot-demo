package frc.robot.subsystems.turret.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  public enum ControlMode {
    POSITION,
    MANUAL_VOLTAGE,
    DISABLED
  }

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final String logKey;

  private ControlMode controlMode = ControlMode.POSITION;
  private double targetPositionRad = HoodConstants.kStorageAngleRad;
  private double manualVolts = 0.0;

  public Hood(HoodIO io, String logKey) {
    super(logKey);
    this.io = io;
    this.logKey = logKey;
  }

  /** Backwards-compatibility escape hatch. Prefer calling subsystem methods directly. */
  public HoodIO getIO() {
    return io;
  }

  /** Sets the hood target in radians and lets periodic() keep applying it. */
  public void setHoodPosition(double positionRad) {
    targetPositionRad =
        MathUtil.clamp(positionRad, HoodConstants.kMinAngleRad, HoodConstants.kMaxAngleRad);
    controlMode = ControlMode.POSITION;
  }

  public void setAngleRad(double positionRad) {
    setHoodPosition(positionRad);
  }

  public void setAngleDeg(double degrees) {
    setHoodPosition(Units.degreesToRadians(degrees));
  }

  public void addAngleDeg(double degrees) {
    setAngleDeg(degrees + Units.radiansToDegrees(targetPositionRad));
  }

  public void setStoragePosition() {
    setHoodPosition(HoodConstants.kStorageAngleRad);
  }

  public void setVoltage(double volts) {
    manualVolts =
        MathUtil.clamp(volts, -HoodConstants.kMaxManualVolts, HoodConstants.kMaxManualVolts);
    controlMode = ControlMode.MANUAL_VOLTAGE;
  }

  public void stop() {
    manualVolts = 0.0;
    controlMode = ControlMode.DISABLED;
  }

  public void zeroAtMin() {
    io.zeroAtMin();
  }

  public boolean isAtGoal() {
    return io.isAtGoal();
  }

  public boolean isZeroed() {
    return io.isZeroed();
  }

  public double getPositionRad() {
    return inputs.positionRad;
  }

  public double getPositionDeg() {
    return Units.radiansToDegrees(inputs.positionRad);
  }

  public double getTargetPositionRad() {
    return targetPositionRad;
  }

  public double getTargetPositionDeg() {
    return Units.radiansToDegrees(targetPositionRad);
  }

  public ControlMode getControlMode() {
    return controlMode;
  }

  @Override
  public void periodic() {

    switch (controlMode) {
      case POSITION:
        if (io.getPaused()) {
          io.setHoodPosition(0);
        } else {
          io.setHoodPosition(targetPositionRad);
        }
        break;
      case MANUAL_VOLTAGE:
        io.setVoltage(manualVolts);
        break;
      case DISABLED:
        io.stop();
        break;
    }

    io.updateInputs(inputs);
    Logger.processInputs(logKey, inputs);
    Logger.recordOutput(logKey + "/ControlMode", controlMode.toString());
    Logger.recordOutput(logKey + "/TargetPositionRad", targetPositionRad);
    Logger.recordOutput(logKey + "/TargetPositionDeg", Units.radiansToDegrees(targetPositionRad));
    Logger.recordOutput(logKey + "/ManualVolts", manualVolts);
    Logger.recordOutput(logKey + "/Paused", io.getPaused());
  }

  public void pause() {

    io.setPaused(true);
  }

  public void resume() {

    io.setPaused(false);
  }
}
