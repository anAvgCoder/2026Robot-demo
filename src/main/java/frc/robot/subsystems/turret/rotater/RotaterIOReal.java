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
  private final StatusSignal<AngularVelocity> absVelSig;

  private final ProfiledPIDController controller;

  private final String logPrefix;

  public RotaterIOReal(int canId, int coderId) {
    motor = new SparkMax(canId, MotorType.kBrushless);
    cancoder = new CANcoder(coderId);

    logPrefix = "Rotater/" + canId + "/";

    absPosSig = cancoder.getAbsolutePosition();
    absVelSig = cancoder.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, absPosSig, absVelSig);
    cancoder.optimizeBusUtilization();

    // Configure motor
    var cfg = new SparkMaxConfig();
    cfg.inverted(RotaterConstants.kInverted)
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

    controller.reset(getMeasuredAngleRad(), getMeasuredVelocityRadPerSec());
  }

  private double getMeasuredAngleRad() {
    absPosSig.refresh();
    double rot = absPosSig.getValueAsDouble();
    return Units.rotationsToRadians(rot);
  }

  private double getMeasuredVelocityRadPerSec() {
    absVelSig.refresh();
    return Units.rotationsToRadians(absVelSig.getValueAsDouble());
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setTurnPosition(double degrees) {
    double clampDeg =
        MathUtil.clamp(degrees, RotaterConstants.kMinAngleDeg, RotaterConstants.kMaxAngleDeg);
    double goalRad = Units.degreesToRadians(clampDeg);

    double measRad = getMeasuredAngleRad();

    double out = controller.calculate(measRad, goalRad);

    // Clamp output to max duty-cycle range
    double capped = MathUtil.clamp(out, -1.0, 1.0);

    // Soft-limit: zero output if at limit and trying to push further
    if (measRad >= RotaterConstants.kMaxAngleRad && capped > 0.0) capped = 0.0;
    if (measRad <= RotaterConstants.kMinAngleRad && capped < 0.0) capped = 0.0;

    motor.set(-capped);

    // Log for tuning
    var sp = controller.getSetpoint();
    Logger.recordOutput(logPrefix + "GoalRad", goalRad);
    Logger.recordOutput(logPrefix + "MeasRad", measRad);
    Logger.recordOutput(logPrefix + "ProfilePosRad", sp.position);
    Logger.recordOutput(logPrefix + "ProfileVelRadPerSec", sp.velocity);
    Logger.recordOutput(logPrefix + "OutputCmd", capped);
  }

  @Override
  public void updateInputs(RotaterIOInputs inputs) {
    inputs.positionRad = getMeasuredAngleRad();
    inputs.velocityRadPerSec = getMeasuredVelocityRadPerSec();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.tempC = motor.getMotorTemperature();
  }
}
