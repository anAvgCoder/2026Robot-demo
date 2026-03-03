package frc.robot.subsystems.turret.rotater;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Rotater extends SubsystemBase implements RotaterIO {
  private final RotaterIO io;
  private final RotaterIOInputsAutoLogged inputs = new RotaterIOInputsAutoLogged();
  private final String logKey;

  public Rotater(RotaterIO io, String logKey) {
    this.io = io;
    this.logKey = logKey;
  }

  public RotaterIO getIO() {
    return this.io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(logKey, inputs);
  }
}
