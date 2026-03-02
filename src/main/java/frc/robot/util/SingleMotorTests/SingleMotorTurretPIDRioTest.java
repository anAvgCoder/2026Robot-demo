package frc.robot.util.SingleMotorTests;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SingleMotorTurretPIDRioTest extends SubsystemBase {

  private final int changeId = System.identityHashCode(this);

  private static final int kTurnMotorCanId = 42;
  private static final int kAbsCancoderCanId = 26;

  private final SparkBase motor;

  private final CANcoder cancoder;
  private final StatusSignal<Angle> absPosSig;

  private final LoggedTunableNumber enabled = new LoggedTunableNumber("TurnRioTest/Enabled", 0.0);
  private final LoggedTunableNumber setpointDeg =
      new LoggedTunableNumber("TurnRioTest/SetpointDeg", 0.0);

  private final LoggedTunableNumber kP = new LoggedTunableNumber("TurnRioTest/kP", 0.25);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("TurnRioTest/kI", 0.00);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("TurnRioTest/kD", 0.00025);

  private final LoggedTunableNumber maxVelRadPerSec =
      new LoggedTunableNumber("TurnRioTest/MaxVelRadPerSec", 502.0);
  private final LoggedTunableNumber maxAccelRadPerSec2 =
      new LoggedTunableNumber("TurnRioTest/MaxAccelRadPerSec2", 1190.0);

  private final LoggedTunableNumber maxOutput =
      new LoggedTunableNumber("TurnRioTest/MaxOutput", 1.0);

  private final ProfiledPIDController profiledPid;

  private boolean wasEnabled = false;

  public SingleMotorTurretPIDRioTest() {

    motor = new SparkMax(kTurnMotorCanId, MotorType.kBrushless);

    cancoder = new CANcoder(kAbsCancoderCanId);
    absPosSig = cancoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, absPosSig);
    cancoder.optimizeBusUtilization();

    var motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0);
    motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(0.1, 0.0, 0.0);
    motorConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(10)
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    profiledPid =
        new ProfiledPIDController(
            kP.get(),
            kI.get(),
            kD.get(),
            new TrapezoidProfile.Constraints(maxVelRadPerSec.get(), maxAccelRadPerSec2.get()));

    profiledPid.setTolerance(Units.degreesToRadians(1.0), Units.degreesToRadians(20.0));
  }

  private double getMeasuredAngleRad() {
    absPosSig.refresh();
    double rot = absPosSig.getValueAsDouble();
    return Units.rotationsToRadians(rot);
  }

  public void setTurnPosition(double goalRad) {
    double meas = getMeasuredAngleRad();
    // Clamp goal to -90° to +90° range
    double goal = MathUtil.clamp(goalRad, -125.0 / 180.0 * Math.PI, 125.0 / 180.0 * Math.PI);

    double out = profiledPid.calculate(meas, goal);

    double capped = MathUtil.clamp(out, -maxOutput.get(), maxOutput.get());
    motor.set(-capped);

    var sp = profiledPid.getSetpoint();
    Logger.recordOutput("TurnRioTest/GoalRad", goal);
    Logger.recordOutput("TurnRioTest/MeasRad", meas);
    Logger.recordOutput("TurnRioTest/ProfilePosRad", sp.position);
    Logger.recordOutput("TurnRioTest/ProfileVelRadPerSec", sp.velocity);
    Logger.recordOutput("TurnRioTest/OutputCmd", capped);
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
        },
        kP,
        kI,
        kD,
        maxVelRadPerSec,
        maxAccelRadPerSec2);
  }

  @Override
  public void periodic() {
    applyTunablesIfChanged();

    double spDegLive = SmartDashboard.getNumber("TurnRioTest/SetpointDeg", setpointDeg.get());
    SmartDashboard.putNumber("TurnRioTest/SetpointDegLive", spDegLive);

    boolean isEnabled = enabled.get() > 0.5;

    double measRad = getMeasuredAngleRad();
    double goalRad = Units.degreesToRadians(spDegLive);

    if (isEnabled && !wasEnabled) {
      profiledPid.reset(measRad);
    }

    if (isEnabled) {
      setTurnPosition(goalRad);
    } else {
      motor.stopMotor();
    }

    double rawRot = absPosSig.getValueAsDouble();
    Logger.recordOutput("TurnRioTest/AbsRotRaw", rawRot);
    Logger.recordOutput("TurnRioTest/PositionRadWrapped", measRad);
    Logger.recordOutput("TurnRioTest/PositionDegWrapped", Units.radiansToDegrees(measRad));

    wasEnabled = isEnabled;
  }
}
