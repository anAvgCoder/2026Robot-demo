package frc.robot.subsystems.intakepivot;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakePivotIOReal implements IntakePivotIO {
  private static final ClosedLoopSlot kPidSlot = ClosedLoopSlot.kSlot0;

  private final SparkMax motor;
  private final RelativeEncoder enc;
  private final SparkClosedLoopController closedLoop;

  private final DigitalInput magSwitch;
  private boolean prevMagState = false;
  private boolean hasBeenZeroed = false;

  private double goalRad = 0.0;

  public IntakePivotIOReal(int canId) {
    motor = new SparkMax(canId, MotorType.kBrushless);
    enc = motor.getEncoder();
    closedLoop = motor.getClosedLoopController();
    magSwitch = new DigitalInput(0);

    SparkMaxConfig cfg = new SparkMaxConfig();

    cfg.idleMode(IdleMode.kBrake)
        .inverted(IntakePivotConstants.kInverted)
        .smartCurrentLimit(IntakePivotConstants.kCurrentLimitAmps)
        .voltageCompensation(12.0);

    cfg.encoder
        .positionConversionFactor(IntakePivotConstants.kPositionFactorRadPerMotorRot)
        .velocityConversionFactor(IntakePivotConstants.kVelocityFactorRadPerSecPerRPM);

    cfg.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

    double maxDuty = MathUtil.clamp(IntakePivotConstants.kMaxVolts / 12.0, 0.0, 1.0);

    cfg.closedLoop
        .pid(IntakePivotConstants.kP, IntakePivotConstants.kI, IntakePivotConstants.kD, kPidSlot)
        .outputRange(-maxDuty, maxDuty, kPidSlot)
        .allowedClosedLoopError(IntakePivotConstants.kPosToleranceRad, kPidSlot);

    tryUntilOk(
        motor,
        5,
        () -> motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    boolean magTriggered = isMagTriggered();
    prevMagState = magTriggered;

    if (magTriggered) {
      enc.setPosition(IntakePivotConstants.kMagSensorPositionRad);
      goalRad = IntakePivotConstants.kMagSensorPositionRad;
      hasBeenZeroed = true;
    } else {
      enc.setPosition(0.0);
      goalRad = 0.0;
    }

    closedLoop.setIAccum(0.0);
  }

  private boolean isMagTriggered() {
    return !magSwitch.get();
  }

  private double getMeasuredPositionRad() {
    return enc.getPosition();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setStoragePosition() {
    if (!hasBeenZeroed) {
      goalRad = IntakePivotConstants.kStoragePosition;
      motor.setVoltage(IntakePivotConstants.kStorageCreepVolts);
      return;
    }
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

    double posRad = getMeasuredPositionRad();
    double errorRad = goalRad - posRad;

    double ffAngleRad = posRad;
    double gravityFFVolts = IntakePivotConstants.kG * Math.cos(ffAngleRad);

    double ksFFVolts = 0.0;
    if (Math.abs(errorRad) > IntakePivotConstants.kPosToleranceRad) {
      ksFFVolts = Math.copySign(IntakePivotConstants.kS, errorRad);
    }

    double ffVolts = gravityFFVolts + ksFFVolts;
    ffVolts =
        MathUtil.clamp(ffVolts, -IntakePivotConstants.kMaxVolts, IntakePivotConstants.kMaxVolts);

    closedLoop.setSetpoint(
        goalRad, SparkBase.ControlType.kPosition, kPidSlot, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public boolean isAtGoal() {
    return Math.abs(goalRad - getMeasuredPositionRad()) < IntakePivotConstants.kPosToleranceRad;
  }

  @Override
  public void zeroToStorage() {
    enc.setPosition(IntakePivotConstants.kStoragePosition);
    goalRad = IntakePivotConstants.kStoragePosition;
    hasBeenZeroed = true;
    closedLoop.setIAccum(0.0);
  }

  @Override
  public void zeroToIntake() {
    enc.setPosition(IntakePivotConstants.kIntakeSecondaryPosition);
    goalRad = IntakePivotConstants.kIntakeSecondaryPosition;
    hasBeenZeroed = true;
    closedLoop.setIAccum(0.0);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    boolean magTriggered = isMagTriggered();
    if (magTriggered && !prevMagState) {
      enc.setPosition(IntakePivotConstants.kMagSensorPositionRad);
      goalRad = IntakePivotConstants.kMagSensorPositionRad;
      hasBeenZeroed = true;
      closedLoop.setIAccum(0.0);
    }
    prevMagState = magTriggered;

    inputs.positionRad = enc.getPosition();
    inputs.velocityRadPerSec = enc.getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.tempC = motor.getMotorTemperature();
    inputs.magSensorTriggered = magTriggered;
    inputs.hasBeenZeroed = hasBeenZeroed;
    inputs.goalPositionRad = goalRad;
  }
}
