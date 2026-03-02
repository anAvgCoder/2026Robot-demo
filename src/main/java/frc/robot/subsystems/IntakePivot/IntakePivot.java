package frc.robot.subsystems.intakepivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  public IntakePivot(IntakePivotIO io) {
    this.io = io;

    // setDefaultCommand(
    //     Commands.run(() -> io.setStoragePosition(), this).withName("HoldStoragePosition"));
  }

  public IntakePivotIO getIO() {
    return this.io;
  }

  public boolean isAtGoal() {
    return io.isAtGoal();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);
  }
}
