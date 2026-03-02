package frc.robot.subsystems.turret.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private ShooterIOInputs inputs = new ShooterIOInputs();

  private final DCMotorSim motorSim;
  private static final DCMotor gearbox = DCMotor.getNeoVortex(1);

  public ShooterIOSim() {
    motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.025, 1), gearbox);
  }

  @Override
  public void setSpeed(double speed) {}

  @Override
  public void hubAdaptiveAiming() {}

  @Override
  public void stopApplyingMotor() {}

  @Override
  public void storageAdaptiveAiming() {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    this.inputs = inputs;
  }
}
