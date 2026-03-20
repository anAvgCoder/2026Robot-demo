package frc.robot.subsystems.intakeRoller;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.SparkUtil;

public class IntakeRollerIOReal implements IntakeRollerIO {
  private final SparkBase motor;
  private final SparkMaxConfig sparkConfig;

  private boolean paused = false;
  private double speed = 0;
  private double prevSpeed = 0;

  public IntakeRollerIOReal() {
    super();
    motor = new SparkMax(IntakeRollerConstants.KCanId, MotorType.kBrushless);

    sparkConfig = new SparkMaxConfig();

    sparkConfig
        .idleMode(SparkMaxConfig.IdleMode.kCoast)
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
    speed = -0.9;
  }

  @Override
  public void outake() {
    speed = 0.8;
  }

  @Override
  public void stop() {
    speed = 0;
  }

  @Override
  public void setPaused(boolean value) {

    this.paused = value;
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {

    //  called from periodic
    if (paused) {

      motor.set(0);
      prevSpeed = 0;
    } else {
      if (prevSpeed != speed) {
        motor.set(speed);
        prevSpeed = speed;
      }
    }

    inputs.supplyCurrent = motor.getOutputCurrent();
    inputs.velocityRPM = motor.getEncoder().getVelocity();
    inputs.tempCelcius = motor.getMotorTemperature();
    inputs.paused = paused;
  }
}
