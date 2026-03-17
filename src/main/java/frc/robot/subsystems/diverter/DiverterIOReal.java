package frc.robot.subsystems.diverter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.SparkUtil;

public class DiverterIOReal implements DiverterIO {
  private final SparkBase motor;
  private final SparkMaxConfig sparkConfig;

  public DiverterIOReal() {
    super();
    motor = new SparkMax(DiverterConstants.KCanId, MotorType.kBrushless);

    sparkConfig = new SparkMaxConfig();

    sparkConfig
        .idleMode(SparkMaxConfig.IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0);
    sparkConfig.signals.appliedOutputPeriodMs(20);

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void intake() {
    motor.set(0.2);
  }

  @Override
  public void outake() {
    motor.set(-0.1);
  }

  @Override
  public void stop() {
    motor.set(0);
  }

  @Override
  public void updateInputs(DiverterIOInputs inputs) {
    inputs.supplyCurrent = motor.getOutputCurrent();
    inputs.velocityRPM = motor.getEncoder().getVelocity();
    inputs.tempCelcius = motor.getMotorTemperature();
  }
}
