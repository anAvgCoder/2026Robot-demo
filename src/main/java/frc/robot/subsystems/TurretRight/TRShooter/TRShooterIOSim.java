package frc.robot.subsystems.turretright.trshooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TRShooterIOSim implements TRShooterIO {
  private TRShooterIOInputs inputs = new TRShooterIOInputs();

  private final DCMotorSim motorSim;
  private static final DCMotor gearbox = DCMotor.getNeoVortex(1);

  public TRShooterIOSim() {
    motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.025, 1), gearbox);
  }

  @Override
  public void setSpeed(double speed) {}

  @Override
  public void hubAdaptiveAiming() {}

  @Override
  public void storageAdaptiveAiming() {}

  @Override
  public void updateInputs(TRShooterIOInputs inputs) {
    this.inputs = inputs;
  }
}
