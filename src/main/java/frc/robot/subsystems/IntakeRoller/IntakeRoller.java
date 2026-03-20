package frc.robot.subsystems.intakeRoller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends SubsystemBase implements IntakeRollerIO {
  private final IntakeRollerIO io;

  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
  }

  public IntakeRollerIO getIO() {
    return this.io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRoller", inputs);
  }

  public void setPaused(boolean value) {
    io.setPaused(value);
  }

  public void pause() {
    io.setPaused(true);
  }

  public void resume() {
    io.setPaused(false);
  }
}
