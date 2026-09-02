package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final SparkFlex motor;
  private final SparkFlexConfig config;

  private double rpm;

  public Shooter(int id) {
    motor = new SparkFlex(id, MotorType.kBrushless);
    config = new SparkFlexConfig();
    rpm = 1000;

    config.idleMode(IdleMode.kCoast).smartCurrentLimit(80);
    config.signals.appliedOutputPeriodMs(20);
    config.inverted(true);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void spit(double rpm) {
    motor.set(rpm / 6784);
  }

  public void nope() {
    motor.set(0);
  }

  public void changeUp(double rpm) {
    this.rpm = this.rpm + rpm;
    motor.set(this.rpm / 6784);
  }
}
