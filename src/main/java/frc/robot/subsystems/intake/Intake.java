package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkMax motor;
  private final SparkMaxConfig sparkConfig;

  public Intake(int id) {
    motor = new SparkMax(id, MotorType.kBrushless);
    sparkConfig = new SparkMaxConfig();

    sparkConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60);
    sparkConfig.signals.appliedOutputPeriodMs(20);
    sparkConfig.inverted(true);

    motor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void suck() {
    motor.set(0.75);
  }

  public void suckFast() {
    motor.set(0.92);
  }

  public void suckSlow() {
    motor.set(0.3);
  }

  public void vomit() {
    motor.set(-0.9);
  }

  public void off() {
    motor.set(0.0);
  }
}
