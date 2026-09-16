package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final SparkFlex motor;
  private final SparkClosedLoopController closedLoop;
  private final RelativeEncoder encoder;

  private double goalRPM = 0.0;
  private boolean closedLoopActive = false;
  private double timeInTolerance = 0.0;

  public Shooter(int id) {
    motor = new SparkFlex(id, MotorType.kBrushless);
    closedLoop = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    SparkFlexConfig config = new SparkFlexConfig();

    config
        .idleMode(IdleMode.kCoast)
        .inverted(kInverted)
        .smartCurrentLimit(kCurrentLimitAmps)
        .voltageCompensation(kNominalVoltage);

    config
        .closedLoop
        .pid(kP, kI, kD, ClosedLoopSlot.kSlot0)
        .outputRange(kMinOutput, kMaxOutput, ClosedLoopSlot.kSlot0);

    config.closedLoop.feedForward.kS(kS, ClosedLoopSlot.kSlot0).kV(kV, ClosedLoopSlot.kSlot0);

    config
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setVelocity(double velocityRPM) {
    goalRPM = Math.max(0.0, velocityRPM);
    closedLoopActive = true;
    closedLoop.setSetpoint(goalRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void setOpenLoop(double percent) {
    goalRPM = 0.0;
    closedLoopActive = false;
    motor.set(MathUtil.clamp(percent, -1.0, 1.0));
  }

  public void stop() {
    goalRPM = 0.0;
    closedLoopActive = false;
    timeInTolerance = 0.0;
    motor.stopMotor();
  }

  public double getVelocityRPM() {
    return encoder.getVelocity();
  }

  public double getErrorRPM() {
    return goalRPM - getVelocityRPM();
  }

  public boolean atGoal() {
    return closedLoopActive && goalRPM > 0.0 && timeInTolerance >= kStableTimeSeconds;
  }

  public Command runAtVelocity(double velocityRPM) {
    return startEnd(() -> setVelocity(velocityRPM), this::stop).withName("Shooter Spin");
  }

  public Command spinUpAndWait(double velocityRPM) {
    return run(() -> setVelocity(velocityRPM)).until(this::atGoal).withName("Shooter SpinUp");
  }

  public Command runOpenLoop(double percent) {
    return startEnd(() -> setOpenLoop(percent), this::stop).withName("Shooter OpenLoop");
  }

  @Override
  public void periodic() {
    double velocityRPM = getVelocityRPM();

    if (closedLoopActive && Math.abs(goalRPM - velocityRPM) <= kToleranceRPM) {
      timeInTolerance += 0.02;
    } else {
      timeInTolerance = 0.0;
    }

    SmartDashboard.putNumber("Shooter/GoalRPM", goalRPM);
    SmartDashboard.putNumber("Shooter/VelocityRPM", velocityRPM);
    SmartDashboard.putNumber("Shooter/ErrorRPM", goalRPM - velocityRPM);
    SmartDashboard.putNumber(
        "Shooter/AppliedVolts", motor.getAppliedOutput() * motor.getBusVoltage());
    SmartDashboard.putNumber("Shooter/CurrentAmps", motor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter/TempC", motor.getMotorTemperature());
    SmartDashboard.putBoolean("Shooter/ClosedLoop", closedLoopActive);
    SmartDashboard.putBoolean("Shooter/AtGoal", atGoal());
  }
}
