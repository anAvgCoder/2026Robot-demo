package frc.robot.subsystems.turret.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class HoodIOSim implements HoodIO {
  private static final double LOOP_PERIOD_SEC = 0.02;

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          HoodConstants.kP,
          HoodConstants.kI,
          HoodConstants.kD,
          new TrapezoidProfile.Constraints(
              HoodConstants.kMaxVelRadPerSec, HoodConstants.kMaxAccelRadPerSec2));

  private double positionRad = HoodConstants.kStorageAngleRad;
  private double velocityRadPerSec = 0.0;
  private double appliedVolts = 0.0;
  private double goalRad = HoodConstants.kStorageAngleRad;
  private boolean closedLoopActive = false;
  private boolean zeroed = HoodConstants.kAssumeMinAngleOnBoot;

  public HoodIOSim() {
    controller.setTolerance(HoodConstants.kPosToleranceRad, HoodConstants.kVelToleranceRadPerSec);
    controller.reset(positionRad, velocityRadPerSec);
  }

  @Override
  public void zeroAtMin() {
    positionRad = HoodConstants.kMinAngleRad;
    velocityRadPerSec = 0.0;
    goalRad = HoodConstants.kMinAngleRad;
    zeroed = true;
    closedLoopActive = false;
    appliedVolts = 0.0;
    controller.reset(positionRad, velocityRadPerSec);
  }

  @Override
  public void setVoltage(double volts) {
    closedLoopActive = false;
    appliedVolts = MathUtil.clamp(volts, -HoodConstants.kMaxManualVolts, HoodConstants.kMaxManualVolts);
    velocityRadPerSec =
        (appliedVolts / HoodConstants.kMaxManualVolts) * HoodConstants.kMaxVelRadPerSec;
    integrate();
  }

  @Override
  public void setHoodPosition(double positionRad) {
    if (!closedLoopActive) {
      controller.reset(this.positionRad, velocityRadPerSec);
    }

    closedLoopActive = true;
    goalRad = MathUtil.clamp(positionRad, HoodConstants.kMinAngleRad, HoodConstants.kMaxAngleRad);

    double appliedDuty =
        MathUtil.clamp(
            controller.calculate(this.positionRad, goalRad),
            -HoodConstants.kMaxClosedLoopDutyCycle,
            HoodConstants.kMaxClosedLoopDutyCycle);

    if (controller.atGoal()) {
      appliedDuty = 0.0;
    }

    appliedVolts = appliedDuty * 12.0;
    velocityRadPerSec = appliedDuty * HoodConstants.kMaxVelRadPerSec;
    integrate();
  }

  @Override
  public void stop() {
    closedLoopActive = false;
    appliedVolts = 0.0;
    velocityRadPerSec = 0.0;
  }

  @Override
  public boolean isAtGoal() {
    return closedLoopActive && controller.atGoal();
  }

  @Override
  public boolean isZeroed() {
    return zeroed;
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.positionRad = positionRad;
    inputs.velocityRadPerSec = velocityRadPerSec;
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = Math.abs(appliedVolts) * 1.5;
    inputs.tempC = 25.0;
    inputs.goalPositionRad = goalRad;
    inputs.goalVelocityRadPerSec = controller.getSetpoint().velocity;
    inputs.closedLoopActive = closedLoopActive;
    inputs.atGoal = isAtGoal();
    inputs.zeroed = zeroed;
  }

  private void integrate() {
    positionRad += velocityRadPerSec * LOOP_PERIOD_SEC;
    positionRad = MathUtil.clamp(positionRad, HoodConstants.kMinAngleRad, HoodConstants.kMaxAngleRad);

    if ((positionRad <= HoodConstants.kMinAngleRad && velocityRadPerSec < 0.0)
        || (positionRad >= HoodConstants.kMaxAngleRad && velocityRadPerSec > 0.0)) {
      velocityRadPerSec = 0.0;
    }
  }
}
