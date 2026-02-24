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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.AutoLogOutputManager;

public class HoodIOReal implements HoodIO {
  private final SparkBase motor;
  private final RelativeEncoder enc;
  private final SparkMaxConfig cfg = new SparkMaxConfig();

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          HoodConstants.kP,
          HoodConstants.kI,
          HoodConstants.kD,
          new TrapezoidProfile.Constraints(
              HoodConstants.kMaxVelRadPerSec, HoodConstants.kMaxAccelRadPerSec2));

  private final ArmFeedforward ff =
      new ArmFeedforward(HoodConstants.kS, HoodConstants.kG, HoodConstants.kV, HoodConstants.kA);

  public HoodIOReal(int canId) {
    motor = new SparkMax(canId, MotorType.kBrushless);
    enc = motor.getEncoder();

    controller.setTolerance(HoodConstants.kPosToleranceRad, HoodConstants.kVelToleranceRadPerSec);

    cfg.idleMode(IdleMode.kBrake)
        .inverted(HoodConstants.kInverted)
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

    zeroAtMin();
  }

  @Override
  public void zeroAtMin() {
    enc.setPosition(0);
    controller.reset(getMeasuredAnglePos(), 0.0);
  }

  private double getMeasuredAnglePos() {
    return enc.getPosition();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setHoodPosition(double position) {
    double goalPos =
        MathUtil.clamp(position, HoodConstants.kMinAnglePos, HoodConstants.kMaxAnglePos);

    double pidVolts = controller.calculate(getMeasuredAnglePos(), goalPos);

    var sp = controller.getSetpoint();
    double ffVolts = ff.calculate(sp.position, sp.velocity);

    double outVolts = pidVolts + ffVolts;
    outVolts = MathUtil.clamp(outVolts, -HoodConstants.kMaxVolts, HoodConstants.kMaxVolts);

    if (getMeasuredAnglePos() >= HoodConstants.kMaxAnglePos && outVolts > 0.0) outVolts = 0.0;
    if (getMeasuredAnglePos() <= HoodConstants.kMinAnglePos && outVolts < 0.0) outVolts = 0.0;
    motor.setVoltage(outVolts);

    AutoLogOutputManager.addObject(sp);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.positionRad = enc.getPosition();
    inputs.velocityRadPerSec = enc.getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.tempC = motor.getMotorTemperature();
  }
}
