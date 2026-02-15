package frc.robot.util.SingleMotorTests;

import static frc.robot.subsystems.Drive.DriveConstants.odometryFrequency;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class SingleModuleTurnPIDRioTest extends SubsystemBase {

  private final int changeId = System.identityHashCode(this);

  private static final int kTurnMotorCanId = 6;
  private static final int kAbsCancoderCanId = 23;

  private final SparkBase motor;
  // private final RelativeEncoder motorRelativeEncoder;
  private final CANcoder cancoder;

  private final PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private final LoggedTunableNumber enabled = new LoggedTunableNumber("TurnRioTest/Enabled", 0.0);
  private final LoggedTunableNumber setpointDeg =
      new LoggedTunableNumber("TurnRioTest/SetpointDeg", 0.0);
  private final LoggedTunableNumber kP = new LoggedTunableNumber("TurnRioTest/kP", 0.0);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("TurnRioTest/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("TurnRioTest/kD", 0.0);

  public SingleModuleTurnPIDRioTest() {

    motor = new SparkMax(kTurnMotorCanId, MotorType.kBrushless);
    cancoder = new CANcoder(kAbsCancoderCanId);

    // motorRelativeEncoder = motor.getEncoder();

    var motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12);
    motorConfig
        .encoder
        .inverted(false)
        .positionConversionFactor(2 * Math.PI)
        .velocityConversionFactor((2 * Math.PI) / 60.0);
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 2 * Math.PI);
    motorConfig
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig,
                com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters));

    pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setTurnPosition(double radians) {
    double setpoint = MathUtil.angleModulus(radians);

    double maxTurnSpeed = 0.5;
    double encoderValue = ((cancoder.getAbsolutePosition().getValueAsDouble() + .5) * 2 * 3.14159);
    double desiredTurnMotorSpeed =
        -MathUtil.clamp(pid.calculate(encoderValue, setpoint), -maxTurnSpeed, maxTurnSpeed);
    motor.set(desiredTurnMotorSpeed);
    System.out.println(desiredTurnMotorSpeed);
  }

  private double getMeasuredAngleRad() {
    return ((cancoder.getAbsolutePosition().getValueAsDouble() + .5) * 2 * 3.14159);
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
        },
        kP,
        kI,
        kD);
  }

  @Override
  public void periodic() {
    applyTunablesIfChanged();

    double spDeg = SmartDashboard.getNumber("TurnRioTest/SetpointDeg", setpointDeg.get());

    double spRad = setpointDegToRad(spDeg);
    double measRad = getMeasuredAngleRad();

    if (enabled.get() > 0.5) {
      setTurnPosition(spRad);
    } else {
      motor.stopMotor();
    }

    double radians = ((cancoder.getAbsolutePosition().getValueAsDouble() + .5) * 2 * 3.14159);
    double deg0to360 = Units.radiansToDegrees(radians);

    Logger.recordOutput("TurnRioTest/PositionDeg", deg0to360);
    Logger.recordOutput("TurnRioTest/PositionRad", radians);
    SmartDashboard.putNumber("TurnRioTest/SetpointDegLive", spDeg);
    Logger.recordOutput("TurnRioTest/MeasRad", measRad);
    Logger.recordOutput("TurnRioTest/SetpointRad", spRad);
  }
}
