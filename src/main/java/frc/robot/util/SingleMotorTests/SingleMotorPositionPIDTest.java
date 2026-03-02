package frc.robot.util.SingleMotorTests;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intakepivot.IntakePivotConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Generic single-motor position PID test using SPARK onboard PID with RIO-side gravity feedforward.
 * Mirrors the IntakePivotPIDRioTest pattern but is meant to be easily adapted for any mechanism by
 * changing the constants at the top.
 */
public class SingleMotorPositionPIDTest extends SubsystemBase {
  private final int changeId = System.identityHashCode(this);

  private static final int kMotorCanId = 40;
  private static final boolean kInverted = true;
  private static final int kCurrentLimitAmps = 10;
  private static final ClosedLoopSlot kPidSlot = ClosedLoopSlot.kSlot0;

  private final SparkMax motor;
  private final RelativeEncoder enc;
  private final SparkClosedLoopController closedLoop;

  private final LoggedTunableNumber enabled =
      new LoggedTunableNumber("SingleMotorPosTest/Enabled", 0.0);

  private final LoggedTunableNumber setpointRad =
      new LoggedTunableNumber(
          "SingleMotorPosTest/SetpointRad", IntakePivotConstants.kStoragePosition);

  private final LoggedTunableNumber zeroEncoder =
      new LoggedTunableNumber("SingleMotorPosTest/ZeroEncoder", 0.0);

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("SingleMotorPosTest/kP", IntakePivotConstants.kP);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("SingleMotorPosTest/kI", IntakePivotConstants.kI);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("SingleMotorPosTest/kD", IntakePivotConstants.kD);

  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("SingleMotorPosTest/kS", IntakePivotConstants.kS);
  private final LoggedTunableNumber kG =
      new LoggedTunableNumber("SingleMotorPosTest/kG", IntakePivotConstants.kG);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("SingleMotorPosTest/kV", IntakePivotConstants.kV);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("SingleMotorPosTest/kA", IntakePivotConstants.kA);

  private final LoggedTunableNumber maxVolts =
      new LoggedTunableNumber("SingleMotorPosTest/MaxVolts", IntakePivotConstants.kMaxVolts);

  private final LoggedTunableNumber kSDeadbandRad =
      new LoggedTunableNumber(
          "SingleMotorPosTest/kSDeadbandRad", IntakePivotConstants.kPosToleranceRad);

  private boolean prevEnabled = false;
  private boolean prevZero = false;

  public SingleMotorPositionPIDTest() {
    motor = new SparkMax(kMotorCanId, MotorType.kBrushless);
    enc = motor.getEncoder();
    closedLoop = motor.getClosedLoopController();

    SmartDashboard.setDefaultNumber("SingleMotorPosTest/SetpointRad", setpointRad.get());

    configureSpark(true);

    enc.setPosition(IntakePivotConstants.kStoragePosition);
    closedLoop.setIAccum(0.0);
  }

  private double getMeasuredPositionRad() {
    return enc.getPosition();
  }

  private double getMeasuredVelocityRadPerSec() {
    return enc.getVelocity();
  }

  private void configureSpark(boolean persist) {
    var cfg = new SparkMaxConfig();

    cfg.idleMode(IdleMode.kBrake)
        .inverted(kInverted)
        .smartCurrentLimit(kCurrentLimitAmps)
        .voltageCompensation(12.0);

    cfg.encoder
        .positionConversionFactor(IntakePivotConstants.kPositionFactorRadPerMotorRot)
        .velocityConversionFactor(IntakePivotConstants.kVelocityFactorRadPerSecPerRPM);

    cfg.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

    double maxDuty = MathUtil.clamp(maxVolts.get() / 12.0, 0.0, 1.0);

    cfg.closedLoop
        .pid(kP.get(), kI.get(), kD.get(), kPidSlot)
        .outputRange(-maxDuty, maxDuty, kPidSlot)
        .allowedClosedLoopError(IntakePivotConstants.kPosToleranceRad, kPidSlot);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                cfg,
                ResetMode.kNoResetSafeParameters,
                persist ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters));
  }

  private void applyTunablesIfChanged() {
    LoggedTunableNumber.ifChanged(
        changeId, () -> configureSpark(false), kP, kI, kD, maxVolts, kS, kG, kV, kA);
  }

  public void setPivotPositionRad(double goalRad) {
    double posRad = getMeasuredPositionRad();
    double errorRad = goalRad - posRad;

    // Scale encoder range [0, -1] → physical angle [0, -π/2]
    double ffAngleRad = posRad * (Math.PI / 2.0);
    double gravityFFVolts = kG.get() * Math.cos(ffAngleRad);

    // Static friction compensation — only applied when error exceeds deadband
    double ksFFVolts = 0.0;
    if (Math.abs(errorRad) > kSDeadbandRad.get()) {
      ksFFVolts = Math.copySign(kS.get(), errorRad);
    }

    double ffVolts = gravityFFVolts + ksFFVolts;
    ffVolts = MathUtil.clamp(ffVolts, -maxVolts.get(), maxVolts.get());

    Logger.recordOutput("SingleMotorPosTest/GravityFFVolts", gravityFFVolts);
    Logger.recordOutput("SingleMotorPosTest/KsFFVolts", ksFFVolts);

    closedLoop.setSetpoint(
        goalRad, SparkBase.ControlType.kPosition, kPidSlot, ffVolts, ArbFFUnits.kVoltage);

    Logger.recordOutput("SingleMotorPosTest/GoalRad", goalRad);
    Logger.recordOutput("SingleMotorPosTest/GoalDeg", Units.radiansToDegrees(goalRad));
    Logger.recordOutput("SingleMotorPosTest/PosRad", posRad);
    Logger.recordOutput("SingleMotorPosTest/PosDeg", Units.radiansToDegrees(posRad));
    Logger.recordOutput("SingleMotorPosTest/ErrorRad", errorRad);
    Logger.recordOutput("SingleMotorPosTest/ErrorDeg", Units.radiansToDegrees(errorRad));
    Logger.recordOutput("SingleMotorPosTest/FFAngleRad", ffAngleRad);
    Logger.recordOutput("SingleMotorPosTest/FFAngleDeg", Units.radiansToDegrees(ffAngleRad));
    Logger.recordOutput("SingleMotorPosTest/FFVolts", ffVolts);

    Logger.recordOutput(
        "SingleMotorPosTest/AppliedVolts", motor.getAppliedOutput() * motor.getBusVoltage());
    Logger.recordOutput("SingleMotorPosTest/SupplyCurrentAmps", motor.getOutputCurrent());
    Logger.recordOutput("SingleMotorPosTest/TempC", motor.getMotorTemperature());
  }

  @Override
  public void periodic() {
    applyTunablesIfChanged();

    double spRadLive =
        SmartDashboard.getNumber("SingleMotorPosTest/SetpointRad", setpointRad.get());
    SmartDashboard.putNumber("SingleMotorPosTest/SetpointRadLive", spRadLive);

    boolean isEnabled = enabled.get() > 0.5;

    boolean zeroNow = zeroEncoder.get() > 0.5;
    if (zeroNow && !prevZero) {
      enc.setPosition(0.0);
      closedLoop.setIAccum(0.0);
      Logger.recordOutput("SingleMotorPosTest/Zeroed", true);
    } else {
      Logger.recordOutput("SingleMotorPosTest/Zeroed", false);
    }
    prevZero = zeroNow;

    if (isEnabled && !prevEnabled) {
      closedLoop.setIAccum(0.0);
    }
    prevEnabled = isEnabled;

    if (isEnabled) {
      double goalRad = setpointRad.get();
      setPivotPositionRad(goalRad);
    } else {
      motor.stopMotor();
    }

    Logger.recordOutput("SingleMotorPosTest/VelocityRadPerSec", getMeasuredVelocityRadPerSec());
    Logger.recordOutput("SingleMotorPosTest/PositionRad", getMeasuredPositionRad());
  }
}
