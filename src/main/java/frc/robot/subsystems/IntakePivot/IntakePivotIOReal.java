package frc.robot.subsystems.intakepivot;

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

public class IntakePivotIOReal implements IntakePivotIO {
  private final SparkBase motor;
  private final RelativeEncoder enc;
  private final SparkMaxConfig cfg = new SparkMaxConfig();

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          IntakePivotConstants.kP,
          IntakePivotConstants.kI,
          IntakePivotConstants.kD,
          new TrapezoidProfile.Constraints(
              IntakePivotConstants.kMaxVelRadPerSec, IntakePivotConstants.kMaxAccelRadPerSec2));

  private final ArmFeedforward ff =
      new ArmFeedforward(
          IntakePivotConstants.kS,
          IntakePivotConstants.kG,
          IntakePivotConstants.kV,
          IntakePivotConstants.kA);

  public IntakePivotIOReal(int canId) {
    motor = new SparkMax(canId, MotorType.kBrushless);
    enc = motor.getEncoder();

    controller.setTolerance(
        IntakePivotConstants.kPosToleranceRad, IntakePivotConstants.kVelToleranceRadPerSec);

    cfg.idleMode(IdleMode.kBrake)
        .inverted(IntakePivotConstants.kInverted)
        .smartCurrentLimit(IntakePivotConstants.kCurrentLimitAmps)
        .voltageCompensation(12.0);

    cfg.encoder
        .positionConversionFactor(IntakePivotConstants.kPositionFactorRadPerMotorRot)
        .velocityConversionFactor(IntakePivotConstants.kVelocityFactorRadPerSecPerRPM);

    cfg.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

    tryUntilOk(
        motor,
        5,
        () -> motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    enc.setPosition(IntakePivotConstants.kStoragePosition);
    controller.reset(enc.getPosition(), 0.0);
  }

  private double getMeasuredPosition() {
    return enc.getPosition();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setStoragePosition() {
    setPivotPosition(IntakePivotConstants.kStoragePosition);
  }

  @Override
  public void setIntakePosition() {
    setPivotPosition(IntakePivotConstants.kIntakePosition);
  }

  @Override
  public void setPivotPosition(double position) {
    double goalPos = position;

    double pidVolts = controller.calculate(getMeasuredPosition(), goalPos);

    var sp = controller.getSetpoint();
    double ffVolts = ff.calculate(sp.position, sp.velocity);

    double outVolts = pidVolts + ffVolts;
    outVolts =
        MathUtil.clamp(outVolts, -IntakePivotConstants.kMaxVolts, IntakePivotConstants.kMaxVolts);

    motor.setVoltage(outVolts);

    AutoLogOutputManager.addObject(sp);
  }

  @Override
  public void zeroToStorage() {
    enc.setPosition(0);
    controller.reset(0, 0.0);
  }

  @Override
  public void zeroToIntake() {
    enc.setPosition(IntakePivotConstants.kIntakePosition);
    controller.reset(IntakePivotConstants.kIntakePosition, 0.0);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.position = enc.getPosition();
    inputs.velocityRPM = enc.getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.tempC = motor.getMotorTemperature();
  }
}
