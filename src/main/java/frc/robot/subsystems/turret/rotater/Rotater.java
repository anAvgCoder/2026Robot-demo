package frc.robot.subsystems.turret.rotater;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Rotater extends SubsystemBase implements RotaterIO {
  private final RotaterIO io;
  private final RotaterIOInputsAutoLogged inputs = new RotaterIOInputsAutoLogged();

  public Rotater(RotaterIO io) {
    this.io = io;
  }

  public RotaterIO getIO() {
    return this.io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRoller", inputs);
  }
}
