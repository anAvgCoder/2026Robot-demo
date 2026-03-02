package frc.robot.subsystems.turret.shooter;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class ShooterIOReal implements ShooterIO {
  private final SparkBase motor;

  private final SparkFlexConfig sparkConfig;
  private final SparkClosedLoopController cl;
  private final RelativeEncoder enc;

  private static final boolean kInverted = true;

  public ShooterIOReal(int CanId) {
    super();
    motor = new SparkFlex(CanId, MotorType.kBrushless);

    cl = motor.getClosedLoopController();
    enc = motor.getEncoder();

    sparkConfig = new SparkFlexConfig();

    sparkConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.shooterMotorCurrentLimit)
        .voltageCompensation(12.0);
    sparkConfig
        .closedLoop
        .pid(ShooterConstants.kP, 0.0, 0.0, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0);
    sparkConfig
        .closedLoop
        .feedForward
        .kS(0.0, ClosedLoopSlot.kSlot0)
        .kV(ShooterConstants.kV, ClosedLoopSlot.kSlot0);
    sparkConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                sparkConfig,
                com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters));
  }

  @Override
  public void setSpeed(double speed) {
    cl.setSetpoint(kInverted ? -speed : speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setOpenSpeed(double speed) {
    motor.set(kInverted ? -speed : speed);
  }

  @Override
  public void stopApplyingMotor() {
    motor.stopMotor();
  }

  @Override
  public void hubAdaptiveAiming() {}

  @Override
  public void storageAdaptiveAiming() {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.supplyCurrent = motor.getOutputCurrent();
    inputs.motorEncoderValue = motor.getEncoder().getPosition();
    inputs.velocityRPM = motor.getEncoder().getVelocity();
    inputs.tempCelcius = motor.getMotorTemperature();
  }
}
