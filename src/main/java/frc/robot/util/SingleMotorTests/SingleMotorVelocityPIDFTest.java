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
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SingleMotorVelocityPIDFTest extends SubsystemBase {
  private static final int kMotorCanId = 15;
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
