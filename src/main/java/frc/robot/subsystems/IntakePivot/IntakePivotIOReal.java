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
import frc.robot.Constants;

public class IntakePivotIOReal implements IntakePivotIO {
  private static final ClosedLoopSlot kPidSlot = ClosedLoopSlot.kSlot0;

  private final SparkMax motor;
  private final RelativeEncoder enc;
  private final SparkClosedLoopController closedLoop;

  private final DigitalInput switchDI;

  public IntakePivotIOReal(int canId, int dioChannel) {
    motor = new SparkMax(canId, MotorType.kBrushless);
    enc = motor.getEncoder();
    closedLoop = motor.getClosedLoopController();
    switchDI = new DigitalInput(dioChannel);

    SparkMaxConfig cfg = new SparkMaxConfig();

    cfg.idleMode(IdleMode.kBrake)
        .inverted(IntakePivotConstants.kInverted)
        .smartCurrentLimit(IntakePivotConstants.kCurrentLimitAmps)
        .voltageCompensation(Constants.kNominalVoltage);

    cfg.encoder
        .positionConversionFactor(IntakePivotConstants.kPositionFactorRadPerMotorRot)
        .velocityConversionFactor(IntakePivotConstants.kVelocityFactorRadPerSecPerRPM);

    cfg.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

    double maxDuty =
        MathUtil.clamp(IntakePivotConstants.kMaxVolts / Constants.kNominalVoltage, 0.0, 1.0);

    cfg.closedLoop
        .pid(IntakePivotConstants.kP, IntakePivotConstants.kI, IntakePivotConstants.kD, kPidSlot)
        .outputRange(-maxDuty, maxDuty, kPidSlot)
        .allowedClosedLoopError(IntakePivotConstants.kPosToleranceRad, kPidSlot);

    tryUntilOk(
        motor,
        5,
        () -> motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    closedLoop.setIAccum(0.0);
  }

  boolean isSwitchOn() {
    // digital io signal is inverted
    return !switchDI.get();
  }

  public boolean seekHome() {
    if (isSwitchOn()) {
      enc.setPosition(IntakePivotConstants.kMagSensorPositionRad);
      seekPosition(IntakePivotConstants.kMagSensorPositionRad);
      return true;
    } else {
      motor.setVoltage(IntakePivotConstants.kStorageCreepVolts);
      return false;
    }
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void seekPosition(double goalRad) {
    double posRad = enc.getPosition();
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
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.velocityRadPerSec = enc.getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.tempC = motor.getMotorTemperature();
    inputs.magSensorTriggered = isSwitchOn();
    inputs.setpoint = closedLoop.getSetpoint();

    inputs.endstop =
        Math.abs(inputs.velocityRadPerSec) < IntakePivotConstants.kVelocityEpsilon
            && inputs.supplyCurrentAmps > IntakePivotConstants.kCurrentEpsilon;

    // Check for conditions in known positions
    if (inputs.magSensorTriggered && inputs.endstop) {
      // enc.setPosition(IntakePivotConstants.kMagSensorPositionRad);
    }
    if (!inputs.magSensorTriggered && inputs.endstop) {
      // enc.setPosition(IntakePivotConstants.kExtendedPosition);
    }

    inputs.positionRad = enc.getPosition();
  }
}
