package frc.robot.subsystems.turret.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.shooter.ShooterIO;

import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase implements HoodIO {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public Hood(HoodIO io) {
    this.io = io;
  }

  public HoodIO getIO() {
    return this.io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  // Optional convenience wrapper; control still happens in IOReal
  public void setAngleDeg(double deg) {
    io.setHoodPosition(deg);
  }
}
