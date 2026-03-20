package frc.robot.subsystems.belt;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Belt extends SubsystemBase implements BeltIO {
  private final BeltIO io;
  private String logkey = "";

  private final BeltIOInputsAutoLogged inputs = new BeltIOInputsAutoLogged();

  public Belt(BeltIO io, String logkey) {
    this.io = io;
    this.logkey = logkey;
  }

  public BeltIO getIO() {
    return this.io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(logkey, inputs);
  }

  public void pause() {
    io.setPaused(true);
  }

  public void resume() {
    io.setPaused(false);
  }
}
