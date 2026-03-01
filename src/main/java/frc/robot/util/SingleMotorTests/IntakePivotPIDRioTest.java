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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intakepivot.IntakePivotConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class IntakePivotPIDRioTest extends SubsystemBase {
  private final int changeId = System.identityHashCode(this);

  private static final int kPivotMotorCanId = IntakePivotConstants.kCanId;
  private static final ClosedLoopSlot kPidSlot = ClosedLoopSlot.kSlot0;

  private final SparkMax motor;
  private final RelativeEncoder enc;
  private final SparkClosedLoopController closedLoop;

  private final LoggedTunableNumber enabled =
      new LoggedTunableNumber("IntakePivotRioTest/Enabled", 0.0);

  private final LoggedTunableNumber setpointDeg =
      new LoggedTunableNumber(
          "IntakePivotRioTest/SetpointDeg",
          Units.radiansToDegrees(IntakePivotConstants.kStoragePosition));

  private final LoggedTunableNumber zeroEncoder =
      new LoggedTunableNumber("IntakePivotRioTest/ZeroEncoder", 0.0);

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("IntakePivotRioTest/kP", IntakePivotConstants.kP);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("IntakePivotRioTest/kI", IntakePivotConstants.kI);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("IntakePivotRioTest/kD", IntakePivotConstants.kD);

  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("IntakePivotRioTest/kS", IntakePivotConstants.kS);
  private final LoggedTunableNumber kG =
      new LoggedTunableNumber("IntakePivotRioTest/kG", IntakePivotConstants.kG);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("IntakePivotRioTest/kV", IntakePivotConstants.kV);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("IntakePivotRioTest/kA", IntakePivotConstants.kA);

  private final LoggedTunableNumber ffAngleOffsetDeg =
      new LoggedTunableNumber("IntakePivotRioTest/FFAngleOffsetDeg", 0.0);

  private final LoggedTunableNumber maxVolts =
      new LoggedTunableNumber("IntakePivotRioTest/MaxVolts", IntakePivotConstants.kMaxVolts);

  private final LoggedTunableNumber kSDeadbandDeg =
      new LoggedTunableNumber("IntakePivotRioTest/kSDeadbandDeg", 1.0);

  private ArmFeedforward ff;

  private boolean prevEnabled = false;
  private boolean prevZero = false;

  public IntakePivotPIDRioTest() {
    motor = new SparkMax(kPivotMotorCanId, MotorType.kBrushless);
    enc = motor.getEncoder();
    closedLoop = motor.getClosedLoopController();

    SmartDashboard.setDefaultNumber("IntakePivotRioTest/SetpointDeg", setpointDeg.get());

    ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    configureSpark(true);

    enc.setPosition(0.0);
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
        .inverted(IntakePivotConstants.kInverted)
        .smartCurrentLimit(IntakePivotConstants.kCurrentLimitAmps)
        .voltageCompensation(12.0);

    cfg.encoder
        .positionConversionFactor(IntakePivotConstants.kPositionFactorRadPerMotorRot)
        .velocityConversionFactor(IntakePivotConstants.kVelocityFactorRadPerSecPerRPM);

    cfg.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

    // Limit the SPARK PID output range (duty cycle). With 12V compensation, duty ~= volts/12.
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
        changeId,
        () -> {
          configureSpark(false);
          ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
        },
        kP,
        kI,
        kD,
        maxVolts,
        kS,
        kG,
        kV,
        kA);
  }

  public void setPivotPositionRad(double goalRad) {

    double posRad = getMeasuredPositionRad();
    double errorRad = goalRad - posRad;

    double ffAngleRad = posRad + Units.degreesToRadians(ffAngleOffsetDeg.get());

    double ksDeadbandRad = Units.degreesToRadians(kSDeadbandDeg.get());
    double velForKsSign =
        (Math.abs(errorRad) > ksDeadbandRad) ? Math.copySign(1e-3, errorRad) : 0.0;

    double ffVolts = ff.calculate(ffAngleRad, velForKsSign);
    ffVolts = MathUtil.clamp(ffVolts, -maxVolts.get(), maxVolts.get());

    closedLoop.setSetpoint(
        goalRad, SparkBase.ControlType.kPosition, kPidSlot, ffVolts, ArbFFUnits.kVoltage);

    Logger.recordOutput("IntakePivotRioTest/GoalRad", goalRad);
    Logger.recordOutput("IntakePivotRioTest/GoalDeg", Units.radiansToDegrees(goalRad));
    Logger.recordOutput("IntakePivotRioTest/PosRad", posRad);
    Logger.recordOutput("IntakePivotRioTest/PosDeg", Units.radiansToDegrees(posRad));
    Logger.recordOutput("IntakePivotRioTest/ErrorRad", errorRad);
    Logger.recordOutput("IntakePivotRioTest/ErrorDeg", Units.radiansToDegrees(errorRad));
    Logger.recordOutput("IntakePivotRioTest/FFAngleRad", ffAngleRad);
    Logger.recordOutput("IntakePivotRioTest/FFAngleDeg", Units.radiansToDegrees(ffAngleRad));
    Logger.recordOutput("IntakePivotRioTest/FFVolts", ffVolts);

    Logger.recordOutput(
        "IntakePivotRioTest/AppliedVolts", motor.getAppliedOutput() * motor.getBusVoltage());
    Logger.recordOutput("IntakePivotRioTest/SupplyCurrentAmps", motor.getOutputCurrent());
    Logger.recordOutput("IntakePivotRioTest/TempC", motor.getMotorTemperature());
  }

  @Override
  public void periodic() {
    applyTunablesIfChanged();

    double spDegLive =
        SmartDashboard.getNumber("IntakePivotRioTest/SetpointDeg", setpointDeg.get());
    SmartDashboard.putNumber("IntakePivotRioTest/SetpointDegLive", spDegLive);

    boolean isEnabled = enabled.get() > 0.5;

    boolean zeroNow = zeroEncoder.get() > 0.5;
    if (zeroNow && !prevZero) {
      enc.setPosition(0.0);
      closedLoop.setIAccum(0.0);
      Logger.recordOutput("IntakePivotRioTest/Zeroed", true);
    } else {
      Logger.recordOutput("IntakePivotRioTest/Zeroed", false);
    }
    prevZero = zeroNow;

    if (isEnabled && !prevEnabled) {
      closedLoop.setIAccum(0.0);
    }
    prevEnabled = isEnabled;

    if (isEnabled) {
      double goalRad = spDegLive;
      setPivotPositionRad(goalRad);
    } else {
      motor.stopMotor();
    }

    Logger.recordOutput("IntakePivotRioTest/VelocityRadPerSec", getMeasuredVelocityRadPerSec());
    Logger.recordOutput("IntakePivotRioTest/PositionDeg", getMeasuredPositionRad());
  }
}
