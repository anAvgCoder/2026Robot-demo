package frc.robot.util.SingleMotorTests;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intakepivot.IntakePivotConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class IntakePivotPIDRioTest extends SubsystemBase {
  private final int changeId = System.identityHashCode(this);

  private static final int kPivotMotorCanId = IntakePivotConstants.kCanId;

  private final SparkBase motor;
  private final RelativeEncoder enc;

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

  private final LoggedTunableNumber maxVelRadPerSec =
      new LoggedTunableNumber(
          "IntakePivotRioTest/MaxVelRadPerSec", IntakePivotConstants.kMaxVelRadPerSec);
  private final LoggedTunableNumber maxAccelRadPerSec2 =
      new LoggedTunableNumber(
          "IntakePivotRioTest/MaxAccelRadPerSec2", IntakePivotConstants.kMaxAccelRadPerSec2);

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

  private final ProfiledPIDController profiledPid;
  private ArmFeedforward ff;

  private boolean prevEnabled = false;
  private boolean prevZero = false;

  public IntakePivotPIDRioTest() {
    motor = new SparkMax(kPivotMotorCanId, MotorType.kBrushless);
    enc = motor.getEncoder();

    var cfg = new SparkMaxConfig();
    cfg.idleMode(IdleMode.kBrake)
        .inverted(IntakePivotConstants.kInverted)
        .smartCurrentLimit(IntakePivotConstants.kCurrentLimitAmps)
        .voltageCompensation(12.0);

    cfg.encoder
        .positionConversionFactor(IntakePivotConstants.kPositionFactorRadPerMotorRot)
        .velocityConversionFactor(IntakePivotConstants.kVelocityFactorRadPerSecPerRPM);

    cfg.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

    tryUntilOk(
        motor,
        5,
        () -> motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    profiledPid =
        new ProfiledPIDController(
            kP.get(),
            kI.get(),
            kD.get(),
            new TrapezoidProfile.Constraints(maxVelRadPerSec.get(), maxAccelRadPerSec2.get()));

    profiledPid.setTolerance(
        IntakePivotConstants.kPosToleranceRad, IntakePivotConstants.kVelToleranceRadPerSec);

    ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    enc.setPosition(0.0);
    profiledPid.reset(enc.getPosition(), 0.0);
  }

  private double getMeasuredPositionRad() {
    return Units.rotationsToRadians(enc.getPosition());
  }

  private void applyTunablesIfChanged() {
    LoggedTunableNumber.ifChanged(
        changeId,
        () -> {
          profiledPid.setP(kP.get());
          profiledPid.setI(kI.get());
          profiledPid.setD(kD.get());
          profiledPid.setConstraints(
              new TrapezoidProfile.Constraints(maxVelRadPerSec.get(), maxAccelRadPerSec2.get()));

          ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
        },
        kP,
        kI,
        kD,
        maxVelRadPerSec,
        maxAccelRadPerSec2,
        kS,
        kG,
        kV,
        kA);
  }

  public void setPivotPositionRad(double goalRad) {
    double measurementRad = getMeasuredPositionRad();

    double pidVolts = profiledPid.calculate(measurementRad, goalRad);

    var sp = profiledPid.getSetpoint();

    double ffAngleRad = sp.position;
    double ffVolts = ff.calculate(ffAngleRad, sp.velocity);

    double outVolts;

    outVolts = MathUtil.clamp(pidVolts + ffVolts, -maxVolts.get(), maxVolts.get());

    motor.setVoltage(outVolts);

    Logger.recordOutput("IntakePivotRioTest/GoalRad", goalRad);
    Logger.recordOutput("IntakePivotRioTest/GoalDeg", Units.radiansToDegrees(goalRad));
    Logger.recordOutput("IntakePivotRioTest/ProfilePosRad", sp.position);
    Logger.recordOutput("IntakePivotRioTest/ProfilePosDeg", Units.radiansToDegrees(sp.position));
    Logger.recordOutput("IntakePivotRioTest/ProfileVelRadPerSec", sp.velocity);
    Logger.recordOutput("IntakePivotRioTest/PIDVolts", pidVolts);
    Logger.recordOutput("IntakePivotRioTest/FFVolts", ffVolts);
    Logger.recordOutput("IntakePivotRioTest/OutVoltsCmd", outVolts);

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
      profiledPid.reset(0.0, 0.0);
      Logger.recordOutput("IntakePivotRioTest/Zeroed", true);
    } else {
      Logger.recordOutput("IntakePivotRioTest/Zeroed", false);
    }
    prevZero = zeroNow;

    if (isEnabled && !prevEnabled) {
      profiledPid.reset(getMeasuredPositionRad(), enc.getVelocity());
    }
    prevEnabled = isEnabled;

    if (isEnabled) {
      double goalRad = Units.degreesToRadians(spDegLive);
      setPivotPositionRad(goalRad);
    } else {
      motor.stopMotor();
    }

    Logger.recordOutput("IntakePivotRioTest/PositionRad", getMeasuredPositionRad());
    Logger.recordOutput(
        "IntakePivotRioTest/PositionDeg", Units.radiansToDegrees(getMeasuredPositionRad()));
    Logger.recordOutput("IntakePivotRioTest/VelocityRadPerSec", enc.getVelocity());
  }
}
