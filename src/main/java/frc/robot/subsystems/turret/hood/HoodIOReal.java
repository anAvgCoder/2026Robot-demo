package frc.robot.subsystems.turret.hood;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.Logger;

public class HoodIOReal implements HoodIO {
  private final SparkBase motor;
  private final RelativeEncoder encoder;
  private final ProfiledPIDController controller;

  private final String logPrefix;

  private double goalRad = HoodConstants.kStorageAngleRad;
  private boolean closedLoopActive = false;
  private boolean zeroed = false;

  public HoodIOReal(int canId) {
    motor = new SparkMax(canId, MotorType.kBrushless);
    encoder = motor.getEncoder();
    logPrefix = "Hood/" + canId;

    var cfg = new SparkMaxConfig();
    cfg.inverted(HoodConstants.kInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(HoodConstants.kCurrentLimitAmps)
        .voltageCompensation(12.0);

    cfg.encoder
        .positionConversionFactor(HoodConstants.kPositionFactorRadPerMotorRot)
        .velocityConversionFactor(HoodConstants.kVelocityFactorRadPerSecPerRPM);

    cfg.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

    tryUntilOk(
        motor,
        5,
        () -> motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    controller =
        new ProfiledPIDController(
            HoodConstants.kP,
            HoodConstants.kI,
            HoodConstants.kD,
            new TrapezoidProfile.Constraints(
                HoodConstants.kMaxVelRadPerSec, HoodConstants.kMaxAccelRadPerSec2));

    controller.setTolerance(HoodConstants.kPosToleranceRad, HoodConstants.kVelToleranceRadPerSec);

    if (HoodConstants.kAssumeMinAngleOnBoot) {
      encoder.setPosition(HoodConstants.kMinAngleRad);
      goalRad = HoodConstants.kMinAngleRad;
      zeroed = true;
    } else {
      goalRad = 0.0;
      zeroed = false;
    }

    controller.reset(encoder.getPosition(), encoder.getVelocity());
  }

  @Override
  public void zeroAtMin() {
    encoder.setPosition(HoodConstants.kMinAngleRad);
    goalRad = HoodConstants.kMinAngleRad;
    closedLoopActive = false;
    zeroed = true;
    controller.reset(HoodConstants.kMinAngleRad, 0.0);
  }

  @Override
  public void setVoltage(double volts) {
    closedLoopActive = false;
    motor.setVoltage(MathUtil.clamp(volts, -HoodConstants.kMaxManualVolts, HoodConstants.kMaxManualVolts));
  }

  @Override
  public void setHoodPosition(double positionRad) {
    double measuredPosition = encoder.getPosition();
    double measuredVelocity = encoder.getVelocity();

    if (!closedLoopActive) {
      controller.reset(measuredPosition, measuredVelocity);
    }

    closedLoopActive = true;
    goalRad = MathUtil.clamp(positionRad, HoodConstants.kMinAngleRad, HoodConstants.kMaxAngleRad);

    double controllerOutputDuty = controller.calculate(measuredPosition, goalRad);

    if (controller.atGoal()) {
      controllerOutputDuty = 0.0;
    }

    double appliedDuty =
        MathUtil.clamp(
            controllerOutputDuty,
            -HoodConstants.kMaxClosedLoopDutyCycle,
            HoodConstants.kMaxClosedLoopDutyCycle);

    motor.set(appliedDuty);

    var setpoint = controller.getSetpoint();
    Logger.recordOutput(logPrefix + "/GoalRad", goalRad);
    Logger.recordOutput(logPrefix + "/MeasuredRad", measuredPosition);
    Logger.recordOutput(logPrefix + "/MeasuredVelRadPerSec", measuredVelocity);
    Logger.recordOutput(logPrefix + "/ProfilePosRad", setpoint.position);
    Logger.recordOutput(logPrefix + "/ProfileVelRadPerSec", setpoint.velocity);
    Logger.recordOutput(logPrefix + "/AppliedDuty", appliedDuty);
  }

  @Override
  public void stop() {
    closedLoopActive = false;
    motor.stopMotor();
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
    inputs.positionRad = encoder.getPosition();
    inputs.velocityRadPerSec = encoder.getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.tempC = motor.getMotorTemperature();
    inputs.goalPositionRad = goalRad;
    inputs.goalVelocityRadPerSec = controller.getSetpoint().velocity;
    inputs.closedLoopActive = closedLoopActive;
    inputs.atGoal = isAtGoal();
    inputs.zeroed = zeroed;
  }
}
