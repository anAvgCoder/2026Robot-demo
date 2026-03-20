package frc.robot.subsystems.intakePivot;

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
  private boolean prevMagState = true;
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

    // digital IO is inverse logic for switches
    if (!magSwitch.get()) {

      // switch is a little outside of zero so set to offset when detected
      enc.setPosition(IntakePivotConstants.kMagSensorPositionRad);
      goalRad = IntakePivotConstants.kMagSensorPositionRad;
      hasBeenZeroed = true;
    }
    //  TODO --- otherwise set to zero - this needs to be looked at
    else {
      enc.setPosition(0.0);
      goalRad = 0.0;
      hasBeenZeroed = false; // this will force a draw in on enablement
    }

    closedLoop.setIAccum(0.0);
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

    // ------------------------------------------------
    // so if we have not been zeroed then set a pull
    //   in voltage until we hit limit switch
    // ------------------------------------------------
    if (!hasBeenZeroed) {
      motor.setVoltage(IntakePivotConstants.kStorageCreepVolts);
    }
    // ---------------------------------------------------------
    // otherwise set the position under position control PID
    // ---------------------------------------------------------
    else {

      setPivotPosition(IntakePivotConstants.kStoragePosition);
    }
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

    // Scale encoder range [0, -1] → physical angle [0, -π/2]
    double ffAngleRad = posRad * (Math.PI / 2.0);
    double gravityFFVolts = IntakePivotConstants.kG * Math.cos(ffAngleRad);

    // Static friction compensation — only applied when error exceeds tolerance
    double ksFFVolts = 0.0; // It's unnecessary from when I last measured
    if (Math.abs(errorRad) > IntakePivotConstants.kPosToleranceRad) {
      ksFFVolts = Math.copySign(IntakePivotConstants.kS, errorRad);
    }

    double ffVolts = gravityFFVolts + ksFFVolts;
    ffVolts =
        MathUtil.clamp(ffVolts, -IntakePivotConstants.kMaxVolts, IntakePivotConstants.kMaxVolts);

    // -----------------------------------------
    // closed loop control to set the position
    // -----------------------------------------
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
    closedLoop.setIAccum(0.0);
  }

  @Override
  public void zeroToIntake() {
    enc.setPosition(IntakePivotConstants.kIntakeSecondaryPosition);
    goalRad = IntakePivotConstants.kIntakeSecondaryPosition;
    closedLoop.setIAccum(0.0);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {

    // digital io in on switches is inverse when
    // active so to have true we need a not clause
    boolean mag = !magSwitch.get();

    // todo need to look at this
    //  this is called from periodic and will reset to zero when needed if not zero
    if (mag && !prevMagState) {
      enc.setPosition(IntakePivotConstants.kMagSensorPositionRad);
      closedLoop.setIAccum(0.0);
      hasBeenZeroed = true;
    }

    //  this gets overwritten in each loop does the logic above do what we need
    prevMagState = mag;

    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.tempC = motor.getMotorTemperature();
    inputs.magSensorTriggered = mag;
    inputs.hasBeenZeroed = hasBeenZeroed;
    inputs.isAtGoal = isAtGoal();
  }
}
