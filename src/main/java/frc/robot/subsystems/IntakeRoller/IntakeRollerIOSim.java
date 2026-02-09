package frc.robot.subsystems.IntakeRoller;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private IntakeRollerIOInputs inputs = new IntakeRollerIOInputs();

  private final DCMotorSim motorSim;
  private static final DCMotor gearbox = DCMotor.getNEO(1);

  public IntakeRollerIOSim() {
    motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.025, 1), gearbox);
  }

  @Override
  public void intake() {
    motorSim.setInputVoltage(12.0 * 0.5);
    inputs.supplyCurrent = 0.5;
    inputs.VelocityRPM = 100.0;
    inputs.tempCelcius = 30.0;
  }

  @Override
  public void outtake() {
    motorSim.setInputVoltage(12.0 * -0.5);
    inputs.supplyCurrent = -0.5;
    inputs.VelocityRPM = -100.0;
    inputs.tempCelcius = 30.0;
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    this.inputs = inputs;
  }
}
