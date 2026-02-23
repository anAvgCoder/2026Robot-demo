package frc.robot.subsystems.turretright.trshooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class TRShooter extends SubsystemBase implements TRShooterIO {
  private final TRShooterIO io;
  private boolean isHubAdaptive = false;
  private boolean isStorageAdaptive = false;

  private final TRShooterIOInputsAutoLogged inputs = new TRShooterIOInputsAutoLogged();

  public TRShooter(TRShooterIO io) {
    this.io = io;
  }

  public TRShooterIO getIO() {
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
    Logger.processInputs("TRShooter", inputs);
  }
}
