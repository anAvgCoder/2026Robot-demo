package frc.robot.subsystems.turret.rotater;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class RotaterIOSim implements RotaterIO {
  private static final double LOOP_PERIOD_SEC = 0.02;

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          RotaterConstants.kP,
          RotaterConstants.kI,
          RotaterConstants.kD,
          new TrapezoidProfile.Constraints(
              RotaterConstants.kMaxVelRadPerSec, RotaterConstants.kMaxAccelRadPerSec2));

  private double positionRad = 0.0;
  private double velocityRadPerSec = 0.0;
  private double appliedVolts = 0.0;
  private double goalRad = 0.0;
  private boolean closedLoopActive = false;

  public RotaterIOSim() {
    controller.setTolerance(
        RotaterConstants.kPosToleranceRad, RotaterConstants.kVelToleranceRadPerSec);
    controller.reset(positionRad, velocityRadPerSec);
  }

  @Override
  public void setVoltage(double volts) {
    closedLoopActive = false;
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    velocityRadPerSec = (appliedVolts / 12.0) * RotaterConstants.kMaxVelRadPerSec;
    integrate();
  }

  @Override
  public void setTurnPosition(double goalDegrees) {
    if (!closedLoopActive) {
      controller.reset(positionRad, velocityRadPerSec);
    }

    closedLoopActive = true;
    goalRad =
        MathUtil.clamp(
            Units.degreesToRadians(goalDegrees),
            RotaterConstants.kMinAngleRad,
            RotaterConstants.kMaxAngleRad);

    appliedVolts =
        MathUtil.clamp(
            controller.calculate(positionRad, goalRad),
            -RotaterConstants.kMaxVolts,
            RotaterConstants.kMaxVolts);

    velocityRadPerSec =
        MathUtil.clamp(
            (appliedVolts / RotaterConstants.kMaxVolts) * RotaterConstants.kMaxVelRadPerSec,
            -RotaterConstants.kMaxVelRadPerSec,
            RotaterConstants.kMaxVelRadPerSec);

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
  public void updateInputs(RotaterIOInputs inputs) {
    inputs.positionRad = positionRad;
    inputs.velocityRadPerSec = velocityRadPerSec;
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = Math.abs(appliedVolts) * 2.0;
    inputs.tempC = 25.0;
    inputs.goalPositionRad = goalRad;
    inputs.goalVelocityRadPerSec = controller.getSetpoint().velocity;
    inputs.closedLoopActive = closedLoopActive;
    inputs.atGoal = isAtGoal();
  }

  private void integrate() {
    positionRad += velocityRadPerSec * LOOP_PERIOD_SEC;
    positionRad =
        MathUtil.clamp(positionRad, RotaterConstants.kMinAngleRad, RotaterConstants.kMaxAngleRad);
  }
}
