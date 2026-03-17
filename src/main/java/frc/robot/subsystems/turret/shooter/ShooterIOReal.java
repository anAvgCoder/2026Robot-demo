package frc.robot.subsystems.turret.shooter;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
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
  private final SparkClosedLoopController closedLoop;
  private final RelativeEncoder encoder;

  private double goalVelocityRPM = 0.0;
  private boolean closedLoopActive = false;

  public ShooterIOReal(int canId) {
    motor = new SparkFlex(canId, MotorType.kBrushless);

    closedLoop = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    sparkConfig = new SparkFlexConfig();

    sparkConfig
        .idleMode(IdleMode.kCoast)
        .inverted(ShooterConstants.kInverted)
        .smartCurrentLimit(ShooterConstants.kCurrentLimitAmps)
        .voltageCompensation(ShooterConstants.kNominalVoltage);

    sparkConfig
        .closedLoop
        .pid(ShooterConstants.kP, 0.0, 0.0, ClosedLoopSlot.kSlot0)
        .outputRange(0, 1.0, ClosedLoopSlot.kSlot0);

    sparkConfig
        .closedLoop
        .feedForward
        .kS(0.001, ClosedLoopSlot.kSlot0)
        .kV(ShooterConstants.kV, ClosedLoopSlot.kSlot0);

    sparkConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void setVelocityRPM(double velocityRPM) {
    goalVelocityRPM = Math.max(0.0, velocityRPM);
    closedLoopActive = true;
    closedLoop.setSetpoint(goalVelocityRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setOpenLoopPercent(double percent) {
    goalVelocityRPM = 0.0;
    closedLoopActive = false;
    motor.set(Math.max(-1.0, Math.min(1.0, percent)));
  }

  @Override
  public void stop() {
    goalVelocityRPM = 0.0;
    closedLoopActive = false;
    motor.stopMotor();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.motorPositionRot = encoder.getPosition();
    inputs.velocityRPM = encoder.getVelocity();
    inputs.tempCelsius = motor.getMotorTemperature();
    inputs.goalVelocityRPM = goalVelocityRPM;
    inputs.closedLoopActive = closedLoopActive;
  }
}
