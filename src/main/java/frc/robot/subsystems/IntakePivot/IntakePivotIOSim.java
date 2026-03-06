package frc.robot.subsystems.intakepivot;

import edu.wpi.first.math.MathUtil;

public class IntakePivotIOSim implements IntakePivotIO {
  private double positionRad = IntakePivotConstants.kStoragePosition;
  private double velocityRadPerSec = 0.0;
  private double appliedVolts = 0.0;
  private double goalRad = IntakePivotConstants.kStoragePosition;
  private boolean hasBeenZeroed = true;
  private boolean magSensorTriggered = true;

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setStoragePosition() {
    setPivotPosition(IntakePivotConstants.kStoragePosition);
  }

  @Override
  public void setIntakePrimaryPosition() {
    setPivotPosition(IntakePivotConstants.kIntakePrimaryPosition);
  }

  @Override
  public void setIntakeSecondaryPosition() {
    setPivotPosition(IntakePivotConstants.kIntakeSecondaryPosition);
  }

  @Override
  public void setPivotPosition(double positionRad) {
    goalRad = positionRad;

    double errorRad = goalRad - positionRad();
    velocityRadPerSec =
        MathUtil.clamp(
            errorRad / 0.02,
            -IntakePivotConstants.kMaxVelRadPerSec,
            IntakePivotConstants.kMaxVelRadPerSec);
    positionRad = positionRad() + velocityRadPerSec * 0.02;
    positionRad =
        MathUtil.clamp(
            positionRad,
            Math.min(
                IntakePivotConstants.kStoragePosition,
                IntakePivotConstants.kIntakeSecondaryPosition),
            Math.max(
                IntakePivotConstants.kStoragePosition,
                IntakePivotConstants.kIntakeSecondaryPosition));

    appliedVolts =
        MathUtil.clamp(
            IntakePivotConstants.kP * (goalRad - positionRad),
            -IntakePivotConstants.kMaxVolts,
            IntakePivotConstants.kMaxVolts);

    magSensorTriggered =
        Math.abs(positionRad - IntakePivotConstants.kMagSensorPositionRad)
            < IntakePivotConstants.kPosToleranceRad;
  }

  @Override
  public boolean isAtGoal() {
    return Math.abs(goalRad - positionRad) < IntakePivotConstants.kPosToleranceRad;
  }

  @Override
  public void zeroToStorage() {
    positionRad = IntakePivotConstants.kStoragePosition;
    goalRad = IntakePivotConstants.kStoragePosition;
    hasBeenZeroed = true;
    magSensorTriggered = true;
  }

  @Override
  public void zeroToIntake() {
    positionRad = IntakePivotConstants.kIntakeSecondaryPosition;
    goalRad = IntakePivotConstants.kIntakeSecondaryPosition;
    hasBeenZeroed = true;
    magSensorTriggered = false;
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.positionRad = positionRad;
    inputs.velocityRadPerSec = velocityRadPerSec;
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = Math.abs(appliedVolts) * 2.0;
    inputs.tempC = 25.0;
    inputs.magSensorTriggered = magSensorTriggered;
    inputs.hasBeenZeroed = hasBeenZeroed;
    inputs.goalPositionRad = goalRad;
  }

  private double positionRad() {
    return positionRad;
  }
}
