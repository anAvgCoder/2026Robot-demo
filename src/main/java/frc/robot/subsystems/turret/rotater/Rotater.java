package frc.robot.subsystems.turret.rotater;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Rotater extends SubsystemBase {
  public enum ControlMode {
    POSITION,
    MANUAL_VOLTAGE,
    DISABLED
  }

  private final RotaterIO io;
  private final RotaterIOInputsAutoLogged inputs = new RotaterIOInputsAutoLogged();
  private final String logKey;

  private ControlMode controlMode = ControlMode.DISABLED;
  private double targetDegrees = 0.0;
  private double manualVolts = 0.0;

  public Rotater(RotaterIO io, String logKey) {
    super(logKey);
    this.io = io;
    this.logKey = logKey;
  }

  public RotaterIO getIO() {
    return io;
  }

  public void setTurnPosition(double degrees) {
    targetDegrees =
        MathUtil.clamp(degrees, RotaterConstants.kMinAngleDeg, RotaterConstants.kMaxAngleDeg);
    controlMode = ControlMode.POSITION;
  }

  public void setTurnPositionRad(double radians) {
    setTurnPosition(Units.radiansToDegrees(radians));
  }

  public void setVoltage(double volts) {
    manualVolts = MathUtil.clamp(volts, -12.0, 12.0);
    controlMode = ControlMode.MANUAL_VOLTAGE;
  }

  public void stop() {
    manualVolts = 0.0;
    controlMode = ControlMode.DISABLED;
  }

  public boolean isAtGoal() {
    return io.isAtGoal();
  }

  public double getPositionRad() {
    return inputs.positionRad;
  }

  public double getPositionDeg() {
    return Units.radiansToDegrees(inputs.positionRad);
  }

  public double getTargetDeg() {
    return targetDegrees;
  }

  public ControlMode getControlMode() {
    return controlMode;
  }

  @Override
  public void periodic() {
    switch (controlMode) {
      case POSITION:
        io.setTurnPosition(targetDegrees);
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
    Logger.recordOutput(logKey + "/TargetDegrees", targetDegrees);
    Logger.recordOutput(logKey + "/ManualVolts", manualVolts);
  }
}
