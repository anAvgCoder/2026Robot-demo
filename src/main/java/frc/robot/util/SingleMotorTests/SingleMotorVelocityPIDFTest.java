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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.shooter.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SingleMotorVelocityPIDFTest extends SubsystemBase {
  private static final int kMotorCanId = 44;
  private static final boolean kInverted = true;

  private final SparkFlex motor = new SparkFlex(kMotorCanId, MotorType.kBrushless);
  private final SparkClosedLoopController cl = motor.getClosedLoopController();
  private final RelativeEncoder enc = motor.getEncoder();

  private final SparkFlexConfig cfg = new SparkFlexConfig();

  private final LoggedTunableNumber enabled =
      new LoggedTunableNumber("SingleMotorTest/Enabled", 0.0);
  private final LoggedTunableNumber setpointRpm =
      new LoggedTunableNumber("SingleMotorTest/SetpointRPM", 0.0);

  private final LoggedTunableNumber useMaxMotionVel =
      new LoggedTunableNumber("SingleMotorTest/UseMAXMotionVelocity", 0.0);
  private final LoggedTunableNumber maxAccelRpmPerSec =
      new LoggedTunableNumber("SingleMotorTest/MaxAccelRPMperSec", 20000.0);

  // Tune these using Daniel's PID Tuning Guide:
  /*
    For the shooter used in 2026, I tuned a PIDF shooter with less than 1rpm oscilation and a 0.1 ms speed correction with these values:
      KV: 0.00184
      P: 0.00006 (increased until oscillations)
      I: 0 (My FF value was accurate enough that I was unnecessary)
      D: 0.0 (it was more than stable enough with just P, don't overtune)

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

  private final LoggedTunableNumber kP = new LoggedTunableNumber("SingleMotorTest/kP", 0.00006);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("SingleMotorTest/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("SingleMotorTest/kD", 0.0);

  // ***** kA only applied in MAXMotion modes. *****
  private final LoggedTunableNumber kS = new LoggedTunableNumber("SingleMotorTest/kS", 0.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("SingleMotorTest/kV", 0.00183);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("SingleMotorTest/kA", 0.0);

  private final LoggedTunableNumber minOut =
      new LoggedTunableNumber("SingleMotorTest/MinOut", -1.0);
  private final LoggedTunableNumber maxOut = new LoggedTunableNumber("SingleMotorTest/MaxOut", 1.0);

  private final int changeId = System.identityHashCode(this);

  public SingleMotorVelocityPIDFTest() {
    cfg.idleMode(IdleMode.kBrake).inverted(kInverted);

    applyConfig(ResetMode.kResetSafeParameters);
  }

  private void applyConfig(ResetMode resetMode) {
    cfg.idleMode(IdleMode.kCoast).smartCurrentLimit(ShooterConstants.shooterMotorCurrentLimit);

    cfg.closedLoop
        .pid(kP.get(), kI.get(), kD.get(), ClosedLoopSlot.kSlot0)
        .outputRange(minOut.get(), maxOut.get(), ClosedLoopSlot.kSlot0);

    cfg.closedLoop
        .feedForward
        .kS(kS.get(), ClosedLoopSlot.kSlot0)
        .kV(kV.get(), ClosedLoopSlot.kSlot0);

    // During tuning: do NOT persist every change (flash wear + slow).
    motor.configure(cfg, resetMode, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        changeId,
        () -> applyConfig(ResetMode.kNoResetSafeParameters),
        kP,
        kI,
        kD,
        kS,
        kV,
        kA,
        minOut,
        maxOut,
        maxAccelRpmPerSec);

    if (enabled.get() > 0.5) {
      double sp = setpointRpm.get();
      ControlType type =
          (useMaxMotionVel.get() > 0.5)
              ? ControlType.kMAXMotionVelocityControl
              : ControlType.kVelocity;

      cl.setSetpoint(sp, type, ClosedLoopSlot.kSlot0);
    } else {
      motor.stopMotor();
    }

    SmartDashboard.putNumber("SingleMotorTest/MeasuredRPM", enc.getVelocity());
    SmartDashboard.putNumber("SingleMotorTest/AppliedOutput", motor.getAppliedOutput());
    SmartDashboard.putNumber("SingleMotorTest/BusVoltage", motor.getBusVoltage());
    SmartDashboard.putNumber("SingleMotorTest/MeasuredRPM", enc.getVelocity());
    SmartDashboard.putNumber("SingleMotorTest/SetpointRPM", setpointRpm.get());
    SmartDashboard.putNumber("SingleMotorTest/UseMAXMotionVelocity", useMaxMotionVel.get());

    Logger.recordOutput("SingleMotorTest/MeasuredRPM", enc.getVelocity());
  }
}
