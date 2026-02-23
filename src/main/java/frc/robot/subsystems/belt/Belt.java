package frc.robot.subsystems.belt;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Belt extends SubsystemBase implements BeltIO {
  private final BeltIO io;

  private final BeltIOInputsAutoLogged inputs = new BeltIOInputsAutoLogged();

  public Belt(BeltIO io) {
    this.io = io;
  }

  public BeltIO getIO() {
    return this.io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("BeltLeft", inputs);
  }
}
