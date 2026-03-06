package frc.robot.subsystems.turret.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private final DCMotorSim motorSim;

  private boolean closedLoopActive = false;
  private double goalVelocityRPM = 0.0;
  private double appliedVolts = 0.0;

  public ShooterIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ShooterConstants.kSimMotor,
                ShooterConstants.kSimJkgMetersSquared,
                ShooterConstants.kSimGearing),
            ShooterConstants.kSimMotor);
  }

  @Override
  public void setVelocityRPM(double velocityRPM) {
    goalVelocityRPM = Math.max(0.0, velocityRPM);
    closedLoopActive = true;
  }

  @Override
  public void setOpenLoopPercent(double percent) {
    goalVelocityRPM = 0.0;
    closedLoopActive = false;
    appliedVolts = MathUtil.clamp(percent, -1.0, 1.0) * ShooterConstants.kNominalVoltage;
  }

  @Override
  public void stop() {
    goalVelocityRPM = 0.0;
    closedLoopActive = false;
    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (closedLoopActive) {
      double velocityErrorRPM = goalVelocityRPM - motorSim.getAngularVelocityRPM();
      double ffVolts =
          goalVelocityRPM
              / ShooterConstants.kNeoVortexFreeSpeedRPM
              * ShooterConstants.kNominalVoltage;
      double fbVolts = velocityErrorRPM * ShooterConstants.kSimVelocityKpVoltsPerRPM;
      appliedVolts =
          MathUtil.clamp(
              ffVolts + fbVolts,
              -ShooterConstants.kNominalVoltage,
              ShooterConstants.kNominalVoltage);
    }

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(ShooterConstants.kLoopPeriodSec);

    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = motorSim.getCurrentDrawAmps();
    inputs.motorPositionRot = motorSim.getAngularPositionRotations();
    inputs.velocityRPM = motorSim.getAngularVelocityRPM();
    inputs.tempCelsius = 25.0;
    inputs.goalVelocityRPM = goalVelocityRPM;
    inputs.closedLoopActive = closedLoopActive;
  }
}
