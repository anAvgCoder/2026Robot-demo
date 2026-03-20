package frc.robot.subsystems.diverter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Diverter extends SubsystemBase implements DiverterIO {
  private final DiverterIO io;

  private final DiverterIOInputsAutoLogged inputs = new DiverterIOInputsAutoLogged();

  public Diverter(DiverterIO io) {
    this.io = io;
  }

  public DiverterIO getIO() {
    return this.io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Diverter", inputs);
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
