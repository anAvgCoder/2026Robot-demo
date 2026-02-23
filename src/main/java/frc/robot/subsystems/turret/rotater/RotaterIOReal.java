package frc.robot.subsystems.turret.rotater;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
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
import org.littletonrobotics.junction.AutoLogOutputManager;

public class RotaterIOReal implements RotaterIO {
  private final SparkBase motor;
  private final RelativeEncoder enc;
  private final SparkMaxConfig cfg = new SparkMaxConfig();

  private final CANcoder turnCAN;
  private final StatusSignal<Angle> absPosSig;

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          RotaterConstants.kP,
          RotaterConstants.kI,
          RotaterConstants.kD,
          new TrapezoidProfile.Constraints(
              RotaterConstants.kMaxVelRadPerSec, RotaterConstants.kMaxAccelRadPerSec2));

  public RotaterIOReal(int canId, int coderId) {
    motor = new SparkMax(canId, MotorType.kBrushless);
    turnCAN = new CANcoder(coderId);

    controller.setTolerance(
        RotaterConstants.kPosToleranceRad, RotaterConstants.kVelToleranceRadPerSec);

    absPosSig = turnCAN.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, absPosSig);
    turnCAN.optimizeBusUtilization();

    enc = motor.getEncoder();

    cfg.idleMode(IdleMode.kBrake)
        .inverted(RotaterConstants.kInverted)
        .smartCurrentLimit(RotaterConstants.kCurrentLimitAmps)
        .voltageCompensation(12.0);
    cfg.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        motor,
        5,
        () -> motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  private double getMeasuredAngleRad() {
    absPosSig.refresh();
    double rot = absPosSig.getValueAsDouble();
    double rad = Units.rotationsToRadians(rot);
    return MathUtil.inputModulus(rad, -Math.PI, Math.PI);
  }
  
  @Override
  public void setTurnPosition(double degrees) {
    double maxOutput = 1.0;

    double clamp =
        MathUtil.clamp(degrees, -RotaterConstants.kMinAngleDeg, RotaterConstants.kMaxAngleDeg);

    double meas = getMeasuredAngleRad();
    double goal = Units.degreesToRadians(clamp);
    double out = controller.calculate(meas, goal);

    double capped = MathUtil.clamp(out, -maxOutput, maxOutput);
    motor.set(-capped);

    var sp = controller.getSetpoint();

    AutoLogOutputManager.addObject(sp);
  }

  @Override
  public void updateInputs(RotaterIOInputs inputs) {
    inputs.positionRad = enc.getPosition();
    inputs.velocityRadPerSec = enc.getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.tempC = motor.getMotorTemperature();
  }
}
