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

  // Utility actions (toggle 0->1 to trigger)
  private final LoggedTunableNumber zeroEncoder =
      new LoggedTunableNumber("IntakePivotRioTest/ZeroEncoder", 0.0);

  // PID tunables
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

  // FF tunables
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("IntakePivotRioTest/kS", IntakePivotConstants.kS);
  private final LoggedTunableNumber kG =
      new LoggedTunableNumber("IntakePivotRioTest/kG", IntakePivotConstants.kG);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("IntakePivotRioTest/kV", IntakePivotConstants.kV);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("IntakePivotRioTest/kA", IntakePivotConstants.kA);

  private final LoggedTunableNumber maxVolts =
      new LoggedTunableNumber("IntakePivotRioTest/MaxVolts", IntakePivotConstants.kMaxVolts);

  private final ProfiledPIDController profiledPid;
  private ArmFeedforward ff;

  private boolean wasEnabled = false;
  private boolean prevZero = false;
  private boolean prevReset = false;

  public IntakePivotPIDRioTest() {
    motor = new SparkMax(kPivotMotorCanId, MotorType.kBrushless);
    enc = motor.getEncoder();

    // Match your real IO config so encoder reads in radians/rad/s
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
            kP.get(), kI.get(), kD.get(), new TrapezoidProfile.Constraints(400, 800));

    profiledPid.setTolerance(
        IntakePivotConstants.kPosToleranceRad, IntakePivotConstants.kVelToleranceRadPerSec);

    ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    enc.setPosition(0);
    profiledPid.reset(enc.getPosition(), 0.0);
  }

  private double getMeasuredPosition() {
    return enc.getPosition();
  }

  public void setPivotPosition(double position) {
    double goalPos = position;

    double pidVolts = profiledPid.calculate(getMeasuredPosition(), goalPos);

    var sp = profiledPid.getSetpoint();
    double ffVolts = ff.calculate(Units.rotationsToRadians(sp.position), sp.velocity);

    double outVolts = pidVolts + ffVolts;
    outVolts = MathUtil.clamp(outVolts, -maxVolts.get(), maxVolts.get());

    motor.setVoltage(outVolts);

    Logger.recordOutput("IntakePivotRioTest/ProfilePosRad", sp.position);
    Logger.recordOutput("IntakePivotRioTest/ProfileVelRadPerSec", sp.velocity);
    Logger.recordOutput("IntakePivotRioTest/PIDVolts", pidVolts);
    Logger.recordOutput("IntakePivotRioTest/FFVolts", ffVolts);
    Logger.recordOutput("IntakePivotRioTest/OutVoltsCmd", outVolts);

    // Basic motor telemetry
    Logger.recordOutput(
        "IntakePivotRioTest/AppliedVolts", motor.getAppliedOutput() * motor.getBusVoltage());
    Logger.recordOutput("IntakePivotRioTest/SupplyCurrentAmps", motor.getOutputCurrent());
    Logger.recordOutput("IntakePivotRioTest/TempC", motor.getMotorTemperature());
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

  @Override
  public void periodic() {
    applyTunablesIfChanged();

    // Live setpoint read (same pattern as your module turn test)
    double spDegLive =
        SmartDashboard.getNumber("IntakePivotRioTest/SetpointDeg", setpointDeg.get());
    SmartDashboard.putNumber("IntakePivotRioTest/SetpointDegLive", spDegLive);

    boolean isEnabled = enabled.get() > 0.5;

    // One-shot actions (trigger on rising edge)

    if (isEnabled) {
      setPivotPosition(spDegLive);
    } else {
      motor.stopMotor();
    }

    Logger.recordOutput("IntakePivotRioTest/Position", getMeasuredPosition());
    Logger.recordOutput("IntakePivotRioTest/VelocityRadPerSec", enc.getVelocity());

    wasEnabled = isEnabled;
  }
}
