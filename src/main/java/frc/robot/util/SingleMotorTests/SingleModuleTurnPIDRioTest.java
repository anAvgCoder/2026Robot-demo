package frc.robot.util.SingleMotorTests;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Turn (azimuth) PID tuning that RUNS ON THE ROBORIO (WPILib PIDController), using a CTRE Phoenix 6
 * CANcoder for absolute angle feedback, and directly commanding the turn motor in OPEN LOOP
 * (voltage).
 *
 * <p>What this is for: - Verify your CANcoder angle wiring/config - Tune kP/kI/kD on the RIO (not
 * the motor controller) - Validate continuous (wrap-around) behavior 0..360
 *
 * <p>What this is NOT: - A final production module controller (you will likely add motion profiling
 * later)
 *
 * <p>Requirements: - Phoenix 6 CANcoder - REV SparkMax controlling the turn motor -
 * LoggedTunableNumber class (6328-style) in frc.robot.util
 */
public class SingleModuleTurnPIDRioTest extends SubsystemBase {

  // ------------------------
  // Hardware IDs (set these)
  // ------------------------
  private static final int kTurnMotorCanId = 6; // TODO set to module turn motor CAN ID
  private static final int kAbsCancoderCanId = 23; // TODO set to module absolute CANcoder CAN ID

  private static final boolean kTurnMotorInverted = false;

  private static final SensorDirectionValue kCancoderDirection =
      SensorDirectionValue.CounterClockwise_Positive;

  private final LoggedTunableNumber enabled = new LoggedTunableNumber("TurnRioTest/Enabled", 0.0);
  private final LoggedTunableNumber setpointDeg =
      new LoggedTunableNumber("TurnRioTest/SetpointDeg", 0.0);
  private final LoggedTunableNumber zeroOffsetDeg =
      new LoggedTunableNumber("TurnRioTest/ZeroOffsetDeg", 0.0);
  private final LoggedTunableNumber kP = new LoggedTunableNumber("TurnRioTest/kP", 4.0);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("TurnRioTest/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("TurnRioTest/kD", 0.0);
  private final LoggedTunableNumber kSVolts = new LoggedTunableNumber("TurnRioTest/kSVolts", 0.0);
  private final LoggedTunableNumber brakeMode =
      new LoggedTunableNumber("TurnRioTest/BrakeMode", 1.0);

  private final SparkMax turnMotor = new SparkMax(kTurnMotorCanId, MotorType.kBrushless);
  private final CANcoder cancoder = new CANcoder(kAbsCancoderCanId);

  private final PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private final int changeId = System.identityHashCode(this);

  public SingleModuleTurnPIDRioTest() {
    var ccCfg = new CANcoderConfiguration();
    var mCfg = new SparkMaxConfig();

    ccCfg.MagnetSensor.SensorDirection = kCancoderDirection;
    cancoder.getConfigurator().apply(ccCfg);

    mCfg.inverted(kTurnMotorInverted);
    mCfg.idleMode((brakeMode.get() > 0.5) ? IdleMode.kBrake : IdleMode.kCoast);
    mCfg.smartCurrentLimit(40);
    turnMotor.configure(mCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pid.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putNumber("TurnRioTest/SetpointDeg", setpointDeg.get());
  }

  private double getMeasuredAngleRad() {
    double rotations = cancoder.getAbsolutePosition().getValueAsDouble();
    double rad = rotations * 2.0 * Math.PI;

    rad -= Units.degreesToRadians(zeroOffsetDeg.get());
    rad = MathUtil.angleModulus(rad);

    return rad;
  }

  private static double setpointDegToRad(double deg) {
    return MathUtil.angleModulus(Units.degreesToRadians(deg));
  }

  private void applyTunablesIfChanged() {
    LoggedTunableNumber.ifChanged(
        changeId,
        () -> {
          pid.setP(kP.get());
          pid.setI(kI.get());
          pid.setD(kD.get());

          var mCfg = new SparkMaxConfig();
          mCfg.inverted(kTurnMotorInverted);
          mCfg.idleMode((brakeMode.get() > 0.5) ? IdleMode.kBrake : IdleMode.kCoast);
          turnMotor.configure(
              mCfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        },
        kP,
        kI,
        kD,
        brakeMode);
  }

  @Override
  public void periodic() {
    applyTunablesIfChanged();

    double spDeg = SmartDashboard.getNumber("TurnRioTest/SetpointDeg", setpointDeg.get());

    double spRad = setpointDegToRad(spDeg);
    double measRad = getMeasuredAngleRad();

    double pidVolts = pid.calculate(measRad, spRad);

    double outVolts = pidVolts;

    if (enabled.get() > 0.5) {
      turnMotor.setVoltage(outVolts);
    } else {
      turnMotor.stopMotor();
      pid.reset();
      outVolts = 0.0;
    }

    SmartDashboard.putNumber("TurnRioTest/MeasDeg", Units.radiansToDegrees(measRad));
    SmartDashboard.putNumber("TurnRioTest/SetpointDegLive", spDeg);
    SmartDashboard.putNumber(
        "TurnRioTest/ErrorDeg", Units.radiansToDegrees(MathUtil.angleModulus(spRad - measRad)));
    SmartDashboard.putNumber("TurnRioTest/OutVolts", outVolts);
    SmartDashboard.putNumber("TurnRioTest/BattV", RobotController.getBatteryVoltage());

    Logger.recordOutput("TurnRioTest/MeasRad", measRad);
    Logger.recordOutput("TurnRioTest/SetpointRad", spRad);
    Logger.recordOutput("TurnRioTest/OutVolts", outVolts);
    Logger.recordOutput("TurnRioTest/ErrorRad", MathUtil.angleModulus(spRad - measRad));
  }
}
