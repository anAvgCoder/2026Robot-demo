package frc.robot.util.SingleMotorTests;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleMotorVelocityPIDFTest extends SubsystemBase {
  private static final int kMotorCanId = 15;
  private static final boolean kInverted = true;

  // Tune these using Daniel's PID Tuning Guide:
  /*
    For the shooter used in 2024, I tuned a PIDF shooter with these values:
      KV: 0.096
      P: 2.08 (increased until oscillations)
      I: 0 (My FF value was accurate enough that I was unnecessary)
      D: 0.047 (started with 1/1000 of P so 0.00208, then increased until it reduced error without causing oscillation)

    Set:
      kA = 0
      kS = 0 (for now)
      Use only kV to get close to target RPM (this is called steady state)

    How to Tune kV:
      Command a few setpoints (example: 2000, 3000, 4000 RPM). (neo vortex max RPM is 6784, so stay below that)
      Adjust kV until the motor holds close to the target RPM with P = 0 (or very small P like 0.0002).
      -If it always runs below target, increase kV.
      -If it always runs above target, decrease kV.

    Then tune PID (start with P only):
      Make sure to set kI = 0, kD = 0

    Increase kP until it recovers quickly when a ball hits the wheel, without oscillating.
      I recommend using the doubling/halving method to get the right value


    Optional improvements:
      If it struggles to hold low RPM accurately, add a small kS.
      If you want faster, more consistent spin-up, add a little kA.

    ** This Feed forward is very important for velocity control, but NOT OFTEN USED for position control like an arm **
  */

  private static final double kP = 0.0002;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static final double kS =
      0.0; // volts the minimum voltage needed to overcome static friction and start the motor
  // moving, important for velocity control as it is the y-intercept of the graph and thus
  // determines how much voltage is applied at low speeds
  private static final double kV =
      0.0; // volts per RPM (by default) most important for velocity control, as it is the slope of
  // the graph and thus determines how much voltage is applied at a given speed
  private static final double kA =
      0.0; // volts per (RPM/s) (by default) likely unnecessary as it is for acceleration

  private final SparkFlex motor = new SparkFlex(kMotorCanId, MotorType.kBrushless);
  private final SparkClosedLoopController controller = motor.getClosedLoopController();
  private final RelativeEncoder encoder = motor.getEncoder();

  public SingleMotorVelocityPIDFTest() {
    var cfg = new SparkFlexConfig().idleMode(IdleMode.kBrake).inverted(kInverted);

    cfg.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0).outputRange(-1.0, 1.0);

    cfg.closedLoop
        .feedForward
        .kS(kS, ClosedLoopSlot.kSlot0)
        .kV(kV, ClosedLoopSlot.kSlot0)
        .kA(kA, ClosedLoopSlot.kSlot0);

    motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setVelocityRpm(double rpm) {
    controller.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public double getRpm() {
    return encoder.getVelocity();
  }

  public void stop() {
    motor.stopMotor();
  }
}
