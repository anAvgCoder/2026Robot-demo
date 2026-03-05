package frc.robot.subsystems.turret.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase implements HoodIO {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final String logKey;

  public Hood(HoodIO io, String logKey) {
    this.io = io;
    this.logKey = logKey;

    // setDefaultCommand(
    //     Commands.run(() -> io.setHoodPosition(0.0), this).withName("HoldStoragePosition"));
  }

  public HoodIO getIO() {
    return this.io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(logKey, inputs);
  }

  // Optional convenience wrapper; control still happens in IOReal
  public void setAngleDeg(double deg) {
    io.setHoodPosition(deg);
  }
}
