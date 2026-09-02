package frc.robot.subsystems.belt;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Belt extends SubsystemBase {
  private final SparkMax motor;
  private final SparkMax motor1;
  private final SparkMaxConfig motorConfig;

  public Belt(int id, int id1) {
    motor = new SparkMax(id, MotorType.kBrushless);
    motor1 = new SparkMax(id1, MotorType.kBrushless);

    motorConfig = new SparkMaxConfig();

    motorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60);
    motorConfig.signals.appliedOutputPeriodMs(20);
    motorConfig.inverted(false);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void convey() {
    motor.set(0.7);
    // motor1.set(0.7);
  }

  public void unconvey() {
    motor.set(-0.7);
    // motor1.set(-0.7);
  }

  public void stop() {
    motor.set(0.0);
    motor1.set(0.0);
  }
}
