package frc.robot.subsystems.belt;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.SparkUtil;

public class BeltIOReal implements BeltIO {
  private final SparkBase motor;
  private final SparkMaxConfig sparkConfig;

  private final SparkClosedLoopController closedLoop;

  private boolean paused = false;

  private double speed = 0;
  private double prevSpeed = 0;

  public BeltIOReal(int canId) {
    super();
    motor = new SparkMax(canId, MotorType.kBrushless);
    sparkConfig = new SparkMaxConfig();

    closedLoop = motor.getClosedLoopController();

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

    sparkConfig.closedLoop.pid(
        BeltConstants.kP, BeltConstants.kI, BeltConstants.kD, ClosedLoopSlot.kSlot0);

    sparkConfig
        .closedLoop
        .feedForward
        .kS(BeltConstants.kS, ClosedLoopSlot.kSlot0)
        .kV(BeltConstants.kV, ClosedLoopSlot.kSlot0);

    sparkConfig.closedLoop.pid(
        BeltConstants.kP, BeltConstants.kI, BeltConstants.kD, ClosedLoopSlot.kSlot0);

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void intake() {

    speed = 5000;

    // motor.set(0.9);
  }

  @Override
  public void outake() {

    speed = -4000;

    // motor.set(-0.8);
  }

  @Override
  public void stop() {

    speed = 0;

    // motor.set(0);
  }

  @Override
  public void setPaused(boolean value) {
    this.paused = value;
  }

  @Override
  public void updateInputs(BeltIO.BeltIOInputs inputs) {

    // called from periodic
    if (paused) {

      motor.set(0);
      prevSpeed = 0; // trigger resend of speed when unpaused
    } else {

      if (prevSpeed != speed) {
        //  check to see if speed has changed from previous loop
        // and only send changes
        closedLoop.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        prevSpeed = speed;
      }
    }

    inputs.supplyCurrent = motor.getOutputCurrent();
    inputs.velocityRPM = motor.getEncoder().getVelocity();
    inputs.tempCelcius = motor.getMotorTemperature();
    inputs.setpointRPM = speed;
    inputs.paused = paused;
  }
}
