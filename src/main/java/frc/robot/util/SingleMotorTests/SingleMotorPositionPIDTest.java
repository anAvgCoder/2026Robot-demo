package frc.robot.util.SingleMotorTests;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

//  Single motor POSITION closed-loop test for turret/turn-wheel style mechanisms.

//  * Notes:
//   - Setpoints are in encoder position units (default: motor rotations).
//   - If you set encoder positionConversionFactor, your setpoint becomes "that unit"
//   - Example: degrees => factor ~ 360/gearRatio.

public class SingleMotorPositionPIDTest extends SubsystemBase {
  private static final int kMotorCanId = 15;
  private static final boolean kInverted = true;

  /**
   * Gear ratio definition used here: gearRatio = motorRotations / mechanismRotations Example: 100:1
   * turret reduction => gearRatio = 100.0
   */
  private static final double kGearRatio = 100.0; // TODO: change for mechanism

  private final SparkFlex motor = new SparkFlex(kMotorCanId, MotorType.kBrushless);
  private final SparkClosedLoopController cl = motor.getClosedLoopController();
  private final RelativeEncoder enc = motor.getEncoder();

  private final SparkFlexConfig cfg = new SparkFlexConfig();

  private final LoggedTunableNumber enabled =
      new LoggedTunableNumber("SingleMotorPosTest/Enabled", 0.0);
  private final LoggedTunableNumber setpointDeg =
      new LoggedTunableNumber("SingleMotorPosTest/SetpointDeg", 0.0);

  private final LoggedTunableNumber kP = new LoggedTunableNumber("SingleMotorPosTest/kP", 0.00);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("SingleMotorPosTest/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("SingleMotorPosTest/kD", 0.0);

  private final LoggedTunableNumber minOut =
      new LoggedTunableNumber("SingleMotorPosTest/MinOut", -1.0);
  private final LoggedTunableNumber maxOut =
      new LoggedTunableNumber("SingleMotorPosTest/MaxOut", 1.0);

  private final LoggedTunableNumber useWrapping =
      new LoggedTunableNumber("SingleMotorPosTest/UseWrapping", 1.0);
  private final LoggedTunableNumber wrapMin =
      new LoggedTunableNumber("SingleMotorPosTest/WrapMin", 0.0);
  private final LoggedTunableNumber wrapMax =
      new LoggedTunableNumber("SingleMotorPosTest/WrapMax", 360.0);

  private final LoggedTunableNumber useSoftLimits =
      new LoggedTunableNumber("SingleMotorPosTest/UseSoftLimits", 0.0);
  private final LoggedTunableNumber softLimitRevDeg =
      new LoggedTunableNumber("SingleMotorPosTest/SoftLimitReverseDeg", -90.0);
  private final LoggedTunableNumber softLimitFwdDeg =
      new LoggedTunableNumber("SingleMotorPosTest/SoftLimitForwardDeg", 90.0);

  // Optional: arbitrary feedforward in volts (useful for stiction/gravity compensation)
  private final LoggedTunableNumber arbFFVolts =
      new LoggedTunableNumber("SingleMotorPosTest/ArbFFVolts", 0.0);

  private final int changeId = System.identityHashCode(this);

  public SingleMotorPositionPIDTest() {
    cfg.idleMode(IdleMode.kBrake).inverted(kInverted);
    applyConfig(ResetMode.kResetSafeParameters);
  }

  private void applyConfig(ResetMode resetMode) {
    // --- Units setup (we want DEGREES at the mechanism) ---
    // Encoder native position is motor rotations. Position factor converts to yourdesired unit.
    double posFactorDegPerMotorRot = 360.0 / kGearRatio;

    // Encoder native velocity is motor RPM. Velocity factor converts to your desired velocity unit.
    // mechanismDegPerSec = motorRPM * (360/gearRatio) / 60 = motorRPM * (6/gearRatio)
    double velFactorDegPerSecPerMotorRPM = 6.0 / kGearRatio;

    cfg.encoder
        .positionConversionFactor(posFactorDegPerMotorRot)
        .velocityConversionFactor(velFactorDegPerSecPerMotorRPM);

    // PID + output range
    cfg.closedLoop
        .pid(kP.get(), kI.get(), kD.get(), ClosedLoopSlot.kSlot0)
        .outputRange(minOut.get(), maxOut.get(), ClosedLoopSlot.kSlot0);

    // --- Wrapping for continuous mechanisms (shortest-path around wrap range) ---
    // positionWrappingEnabled + inputRange exist on ClosedLoopConfig.
    boolean wrapping = useWrapping.get() > 0.5;
    cfg.closedLoop.positionWrappingEnabled(wrapping);
    if (wrapping) {
      cfg.closedLoop.positionWrappingInputRange(wrapMin.get(), wrapMax.get());
    }

    // --- Soft limits
    boolean softLimits = useSoftLimits.get() > 0.5;
    cfg.softLimit
        .reverseSoftLimit(softLimitRevDeg.get())
        .forwardSoftLimit(softLimitFwdDeg.get())
        .reverseSoftLimitEnabled(softLimits)
        .forwardSoftLimitEnabled(softLimits);

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
        minOut,
        maxOut,
        useWrapping,
        wrapMin,
        wrapMax,
        useSoftLimits,
        softLimitRevDeg,
        softLimitFwdDeg);

    if (enabled.get() > 0.5) {
      double spDeg = setpointDeg.get();

      ControlType type = ControlType.kPosition;

      cl.setSetpoint(spDeg, type, ClosedLoopSlot.kSlot0, arbFFVolts.get(), ArbFFUnits.kVoltage);
    } else {
      motor.stopMotor();
    }

    // Telemetry (degrees and degrees/sec because of conversion factors)
    SmartDashboard.putNumber("SingleMotorPosTest/PosDeg", enc.getPosition());
    SmartDashboard.putNumber("SingleMotorPosTest/VelDegPerSec", enc.getVelocity());
    SmartDashboard.putNumber("SingleMotorPosTest/AppliedOutput", motor.getAppliedOutput());
    SmartDashboard.putNumber("SingleMotorPosTest/BusVoltage", motor.getBusVoltage());
    SmartDashboard.putNumber("SingleMotorPosTest/SetpointDeg", setpointDeg.get());

    Logger.recordOutput("SingleMotorPosTest/PosDeg", enc.getPosition());
    Logger.recordOutput("SingleMotorPosTest/VelDegPerSec", enc.getVelocity());
  }
}
