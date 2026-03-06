package frc.robot.subsystems.intakepivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  public enum Target {
    STORAGE,
    INTAKE_PRIMARY,
    INTAKE_SECONDARY,
    MANUAL_VOLTAGE,
    DISABLED
  }

  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  private Target target = Target.STORAGE;
  private double manualVolts = 0.0;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
  }

  public void setTarget(Target target) {
    this.target = target;
  }

  public Target getTarget() {
    return target;
  }

  public void setStoragePosition() {
    target = Target.STORAGE;
  }

  public void setIntakePrimaryPosition() {
    target = Target.INTAKE_PRIMARY;
  }

  public void setIntakeSecondaryPosition() {
    target = Target.INTAKE_SECONDARY;
  }

  public void setPivotPosition(double positionRad) {
    target = Target.DISABLED;
    io.setPivotPosition(positionRad);
  }

  public void setManualVoltage(double volts) {
    manualVolts = volts;
    target = Target.MANUAL_VOLTAGE;
  }

  public void stop() {
    manualVolts = -1.0;
    target = Target.DISABLED;
  }

  public boolean isAtGoal() {
    return io.isAtGoal();
  }

  public void zeroToStorage() {
    io.zeroToStorage();
  }

  public void zeroToIntake() {
    io.zeroToIntake();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);
    Logger.recordOutput("IntakePivot/Target", target.toString());
    Logger.recordOutput("IntakePivot/ManualVolts", manualVolts);

    switch (target) {
      case STORAGE:
        io.setStoragePosition();
        break;
      case INTAKE_PRIMARY:
        io.setIntakePrimaryPosition();
        break;
      case INTAKE_SECONDARY:
        io.setIntakeSecondaryPosition();
        break;
      case MANUAL_VOLTAGE:
        io.setVoltage(manualVolts);
        break;
      case DISABLED:
        io.setVoltage(0.0);
        break;
    }
  }
}
