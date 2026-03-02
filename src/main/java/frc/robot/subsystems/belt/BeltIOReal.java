package frc.robot.subsystems.belt;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.SparkUtil;

public class BeltIOReal implements BeltIO {
  private final SparkBase motor;
  private final SparkMaxConfig sparkConfig;

  public BeltIOReal(int canId) {
    super();
    motor = new SparkMax(canId, MotorType.kBrushless);
    sparkConfig = new SparkMaxConfig();

    if (canId == BeltConstants.CanIdLeft) {
      sparkConfig
          .idleMode(SparkMaxConfig.IdleMode.kCoast)
          .inverted(BeltConstants.kLeftInverted)
          .smartCurrentLimit(40)
          .voltageCompensation(12.0);
    } else {
      sparkConfig
          .idleMode(SparkMaxConfig.IdleMode.kCoast)
          .inverted(BeltConstants.kRightInverted)
          .smartCurrentLimit(40)
          .voltageCompensation(12.0);
    }

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
    motor.set(0.5);
  }

  @Override
  public void outake() {
    motor.set(-0.5);
  }

  @Override
  public void stop() {
    motor.set(0);
  }

  @Override
  public void updateInputs(BeltIO.BeltIOInputs inputs) {
    inputs.supplyCurrent = motor.getOutputCurrent();
    inputs.velocityRPM = motor.getEncoder().getVelocity();
    inputs.tempCelcius = motor.getMotorTemperature();
  }
}
