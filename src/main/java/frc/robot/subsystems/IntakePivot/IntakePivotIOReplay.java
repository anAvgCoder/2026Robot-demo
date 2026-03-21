package frc.robot.subsystems.intakePivot;

public class IntakePivotIOReplay implements IntakePivotIO {
  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {}

  @Override
  public void setVoltage(double volts) {}

  @Override
  public void seekPosition(double positionRad) {}

  @Override
  public boolean seekHome() {
    return true;
  }
}
