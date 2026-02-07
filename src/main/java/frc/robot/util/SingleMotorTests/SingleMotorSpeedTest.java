package frc.robot.util.SingleMotorTests;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleMotorSpeedTest extends SubsystemBase {
  private static final int kMotorCanId = 15; /** Change to CAN ID of motor being tested */
  private static final boolean kInverted = true;

  private final SparkFlex motor = new SparkFlex(kMotorCanId, MotorType.kBrushless); // Neo Vortex
  // private final SparkMax motor = new SparkMax(kMotorCanId, MotorType.kBrushless); // Neo and Neo550


  public SingleMotorSpeedTest() {
    /* Neo flex smart current values are set in client*/
    var motorCfg = 
      new SparkFlexConfig()
          .idleMode(IdleMode.kBrake);

    motor.configure(motorCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
  }

  public void setSpeed(double speed) {
    if (kInverted) {
      speed = -speed;
    }
    motor.set(speed);
  }

    public double getRpm() {
    return motor.getEncoder().getVelocity();
  }

  public void stop() {
    motor.set(0.0);
  }
}
