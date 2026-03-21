package frc.robot.subsystems.intakePivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  public enum Target {
    STORAGE,
    EXTENDED,
    MANUAL_POSITION,
    MANUAL_VOLTAGE,
    DISABLED
  }

  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  @AutoLogOutput private Target target = Target.STORAGE;
  @AutoLogOutput private double manualVolts = 0.0;
  @AutoLogOutput private double goalRad = IntakePivotConstants.kMagSensorPositionRad;
  @AutoLogOutput private boolean hasBeenZeroed = false;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
  }

  public void setStoragePosition() {
    target = Target.STORAGE;
    goalRad = IntakePivotConstants.kStoragePosition;
  }

  public void setIntakeExtended() {
    target = Target.EXTENDED;
    goalRad = IntakePivotConstants.kExtendedPosition;
  }

  public void setManualPosition(double positionRad) {
    target = Target.MANUAL_POSITION;
    goalRad = positionRad;
  }

  public void setManualVoltage(double volts) {
    target = Target.MANUAL_VOLTAGE;
    manualVolts = volts;
  }

  public void stop() {
    manualVolts = -1.0;
    target = Target.DISABLED;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    if (!hasBeenZeroed) {
      hasBeenZeroed = this.io.seekHome();
      if (!hasBeenZeroed) {
        // must be zeroed before we can start obeying intent
        return;
      }
    }

    switch (target) {
      case STORAGE:
      case EXTENDED:
      case MANUAL_POSITION:
        io.seekPosition(goalRad);
        break;
      case MANUAL_VOLTAGE:
        io.setVoltage(manualVolts);
        break;
      case DISABLED:
      default:
        io.setVoltage(0.0);
        break;
    }
  }

  @AutoLogOutput
  public boolean isAtGoal() {
    return Math.abs(goalRad - inputs.positionRad) < IntakePivotConstants.kPosToleranceRad;
  }
}
