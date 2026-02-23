package frc.robot.subsystems.turretleft.tlshooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class TLShooter extends SubsystemBase implements TLShooterIO {
  private final TLShooterIO io;
  private boolean isHubAdaptive = false;
  private boolean isStorageAdaptive = false;

  private final TLShooterIOInputsAutoLogged inputs = new TLShooterIOInputsAutoLogged();

  public TLShooter(TLShooterIO io) {
    this.io = io;
  }

  public TLShooterIO getIO() {
    return this.io;
  }

  public void startHubAdaptiveAiming() {
    isHubAdaptive = true;
    isStorageAdaptive = false;
  }

  public void startStorageAdaptiveAiming() {
    isHubAdaptive = false;
    isStorageAdaptive = true;
  }

  public void periodic() {
    // Todo: logic to have adaptive hub and storage aiming

    io.updateInputs(inputs);
    Logger.processInputs("TLShooter", inputs);
  }
}
