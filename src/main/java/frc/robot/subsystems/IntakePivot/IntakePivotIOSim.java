package frc.robot.subsystems.intakePivot;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class IntakePivotIOSim implements IntakePivotIO {
  private double positionRad = IntakePivotConstants.kStoragePosition;

  private double voltageSetpoint = -1;
  private double positionSetpoint = -1;

  @Override
  public void setVoltage(double volts) {
    voltageSetpoint = MathUtil.clamp(volts, -12.0, 12.0);
    positionSetpoint = -1;
  }

  @Override
  public void seekPosition(double goalRad) {
    positionSetpoint = goalRad;
    voltageSetpoint = -1;
  }

  @Override
  public boolean seekHome() {
    positionRad = IntakePivotConstants.kMagSensorPositionRad;
    return true;
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    // TODO: I believe there are sim classes that does all this for us
    double velocityRadPerSec = 0, appliedVolts = 0;

    if (positionSetpoint >= 0) {
      double errorRad = positionSetpoint - positionRad;
      velocityRadPerSec =
          MathUtil.clamp(
              errorRad / 0.02,
              -IntakePivotConstants.kMaxVelRadPerSec,
              IntakePivotConstants.kMaxVelRadPerSec);
      positionRad = positionRad + velocityRadPerSec * 0.02;
      positionRad =
          MathUtil.clamp(
              positionRad,
              Math.min(
                  IntakePivotConstants.kStoragePosition, IntakePivotConstants.kExtendedPosition),
              Math.max(
                  IntakePivotConstants.kStoragePosition, IntakePivotConstants.kExtendedPosition));

      appliedVolts =
          MathUtil.clamp(
              IntakePivotConstants.kP * (positionSetpoint - positionRad),
              -IntakePivotConstants.kMaxVolts,
              IntakePivotConstants.kMaxVolts);
    } else if (voltageSetpoint >= 0) {
      velocityRadPerSec =
          IntakePivotConstants.kMaxVelRadPerSec * voltageSetpoint / Constants.kNominalVoltage;
      appliedVolts = voltageSetpoint;
    }

    boolean magSensorTriggered =
        Math.abs(positionRad - IntakePivotConstants.kMagSensorPositionRad)
            < IntakePivotConstants.kPosToleranceRad;

    inputs.positionRad = positionRad;
    inputs.velocityRadPerSec = velocityRadPerSec;
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = Math.abs(appliedVolts) * 2.0;
    inputs.tempC = 25.0;
    inputs.magSensorTriggered = magSensorTriggered;
    inputs.setpoint = positionSetpoint;
  }
}
