package frc.robot.subsystems.turret.rotater;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.Logger;

public class RotaterIOReal implements RotaterIO {
  private final SparkBase motor;
  private final CANcoder cancoder;
  private final StatusSignal<Angle> absPosSig;
  private final StatusSignal<AngularVelocity> velSig;
  private final ProfiledPIDController controller;

  private final String logPrefix;
  private final double encoderOffsetRad;
  private final double closedLoopOutputSign;

  private double goalRad = 0.0;
  private boolean closedLoopActive = false;

  public RotaterIOReal(int canId, int coderId) {
    motor = new SparkMax(canId, MotorType.kBrushless);
    cancoder = new CANcoder(coderId);

    logPrefix = "Rotater/" + canId;
    encoderOffsetRad =
        canId == RotaterConstants.kCanIdRight
            ? RotaterConstants.kAbsEncoderOffsetRightRad
            : RotaterConstants.kAbsEncoderOffsetLeftRad;
    closedLoopOutputSign =
        canId == RotaterConstants.kCanIdRight
            ? RotaterConstants.kClosedLoopOutputSignRight
            : RotaterConstants.kClosedLoopOutputSignLeft;

    absPosSig = cancoder.getAbsolutePosition();
    velSig = cancoder.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, absPosSig, velSig);
    cancoder.optimizeBusUtilization();

    var cfg = new SparkMaxConfig();
    cfg.inverted(
            canId == RotaterConstants.kCanIdRight
                ? RotaterConstants.kInvertedRight
                : RotaterConstants.kInvertedLeft)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(RotaterConstants.kCurrentLimitAmps)
        .voltageCompensation(12.0);

    cfg.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

    tryUntilOk(
        motor,
        5,
        () -> motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    controller =
        new ProfiledPIDController(
            RotaterConstants.kP,
            RotaterConstants.kI,
            RotaterConstants.kD,
            new TrapezoidProfile.Constraints(
                RotaterConstants.kMaxVelRadPerSec, RotaterConstants.kMaxAccelRadPerSec2));

    controller.setTolerance(
        RotaterConstants.kPosToleranceRad, RotaterConstants.kVelToleranceRadPerSec);

    double measuredPosition = getMeasuredAngleRad();
    double measuredVelocity = getMeasuredVelocityRadPerSec();
    goalRad = measuredPosition;
    controller.reset(measuredPosition, measuredVelocity);
  }

  @Override
  public void setVoltage(double volts) {
    closedLoopActive = false;
    motor.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
  }

  @Override
  public void setTurnPosition(double goalDegrees) {
    double measuredPosition = getMeasuredAngleRad();
    double measuredVelocity = getMeasuredVelocityRadPerSec();

    if (!closedLoopActive) {
      controller.reset(measuredPosition, measuredVelocity);
    }

    closedLoopActive = true;
    goalRad =
        MathUtil.clamp(
            Units.degreesToRadians(goalDegrees),
            RotaterConstants.kMinAngleRad,
            RotaterConstants.kMaxAngleRad);

    double controllerOutputVolts = controller.calculate(measuredPosition, goalRad);
    double appliedVolts =
        MathUtil.clamp(
            closedLoopOutputSign * controllerOutputVolts,
            -RotaterConstants.kMaxVolts,
            RotaterConstants.kMaxVolts);

    motor.setVoltage(appliedVolts);

    var setpoint = controller.getSetpoint();
    Logger.recordOutput(logPrefix + "/GoalRad", goalRad);
    Logger.recordOutput(logPrefix + "/MeasuredRad", measuredPosition);
    Logger.recordOutput(logPrefix + "/MeasuredVelRadPerSec", measuredVelocity);
    Logger.recordOutput(logPrefix + "/ProfilePosRad", setpoint.position);
    Logger.recordOutput(logPrefix + "/ProfileVelRadPerSec", setpoint.velocity);
    Logger.recordOutput(logPrefix + "/AppliedClosedLoopVolts", appliedVolts);
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
  public void updateInputs(RotaterIOInputs inputs) {
    double measuredPosition = getMeasuredAngleRad();
    double measuredVelocity = getMeasuredVelocityRadPerSec();

    inputs.positionRad = measuredPosition;
    inputs.velocityRadPerSec = measuredVelocity;
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.tempC = motor.getMotorTemperature();
    inputs.goalPositionRad = goalRad;
    inputs.goalVelocityRadPerSec = controller.getSetpoint().velocity;
    inputs.closedLoopActive = closedLoopActive;
    inputs.atGoal = isAtGoal();
  }

  private double getMeasuredAngleRad() {
    BaseStatusSignal.refreshAll(absPosSig, velSig);
    double absoluteRotations = absPosSig.getValueAsDouble();
    double angleRad = Units.rotationsToRadians(absoluteRotations) - encoderOffsetRad;
    return MathUtil.angleModulus(angleRad);
  }

  private double getMeasuredVelocityRadPerSec() {
    BaseStatusSignal.refreshAll(absPosSig, velSig);
    return Units.rotationsToRadians(velSig.getValueAsDouble());
  }
}
