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
  private final RelativeEncoder enc;

  private final ProfiledPIDController controller;

  private final String logPrefix;
  private double lastGoalRad = Double.NaN;

  public HoodIOReal(int canId) {
    motor = new SparkMax(canId, MotorType.kBrushless);
    enc = motor.getEncoder();

    logPrefix = "Hood/" + canId + "/";

    // Configure motor
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

    // RIO-side profiled PID using motor relative encoder
    controller =
        new ProfiledPIDController(
            HoodConstants.kP,
            HoodConstants.kI,
            HoodConstants.kD,
            new TrapezoidProfile.Constraints(
                HoodConstants.kMaxVelRadPerSec, HoodConstants.kMaxAccelRadPerSec2));

    controller.setTolerance(HoodConstants.kPosToleranceRad, HoodConstants.kVelToleranceRadPerSec);

    // Assume hood starts at min position on boot
    enc.setPosition(HoodConstants.kMinAngleRad);
    controller.reset(HoodConstants.kMinAngleRad, 0.0);
  }

  @Override
  public void zeroAtMin() {
    enc.setPosition(HoodConstants.kMinAngleRad);
    controller.reset(HoodConstants.kMinAngleRad, 0.0);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
    // Invalidate so the controller resets on the next setHoodPosition call
    lastGoalRad = Double.NaN;
  }

  @Override
  public void setHoodPosition(double position) {
    double meas = enc.getPosition();
    double goal = MathUtil.clamp(position, HoodConstants.kMinAngleRad, HoodConstants.kMaxAngleRad);

    if (Double.isNaN(lastGoalRad)
        || Math.abs(goal - lastGoalRad) > HoodConstants.kPosToleranceRad) {
      if (Double.isNaN(lastGoalRad)) {
        // First call since boot / idle — reset profile to current state
        controller.reset(meas, 0.0);
      }
      controller.setGoal(goal);
      lastGoalRad = goal;
    }

    double out = controller.calculate(meas);

    if (controller.atGoal()) {
      out = 0.0;
    }

    double capped = MathUtil.clamp(out, -HoodConstants.kMaxOutput, HoodConstants.kMaxOutput);

    // if (meas >= HoodConstants.kMaxAngleRad && capped < 0.0) capped = 0.0;
    // if (meas <= HoodConstants.kMinAngleRad && capped > 0.0) capped = 0.0;

    motor.set(capped);

    var sp = controller.getSetpoint();
    Logger.recordOutput(logPrefix + "GoalRad", goal);
    Logger.recordOutput(logPrefix + "MeasRad", meas);
    Logger.recordOutput(logPrefix + "ProfilePosRad", sp.position);
    Logger.recordOutput(logPrefix + "ProfileVelRadPerSec", sp.velocity);
    Logger.recordOutput(logPrefix + "OutputCmd", capped);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.position = enc.getPosition();
    inputs.velocityRPM = enc.getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.tempC = motor.getMotorTemperature();
  }
}
