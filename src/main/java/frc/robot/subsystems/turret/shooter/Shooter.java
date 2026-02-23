package frc.robot.subsystems.turret.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase implements ShooterIO {
  private final ShooterIO io;
  private boolean isHubAdaptive = false;
  private boolean isStorageAdaptive = false;

  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public ShooterIO getIO() {
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
    Logger.processInputs("Shooter", inputs);
  }
}
