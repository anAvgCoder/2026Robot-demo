// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import java.util.Queue;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutputManager;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOSpark implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final SparkBase driveSpark;
  private final SparkBase turnSpark;
  private final RelativeEncoder driveEncoder;

  private final CANcoder turnCAN;
  private final StatusSignal<Angle> absPosSig;

  private final ProfiledPIDController turnMotorPIDController =
      new ProfiledPIDController(0.25, 0, 0.00025, new TrapezoidProfile.Constraints(502, 1190));

  // Closed loop controllers
  private final SparkClosedLoopController driveController;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ModuleIOSpark(int module) {
    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> new Rotation2d();
        };
    driveSpark =
        new SparkFlex(
            switch (module) {
              case 0 -> frontLeftDriveCanId;
              case 1 -> frontRightDriveCanId;
              case 2 -> backLeftDriveCanId;
              case 3 -> backRightDriveCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    turnSpark =
        new SparkMax(
            switch (module) {
              case 0 -> frontLeftTurnCanId;
              case 1 -> frontRightTurnCanId;
              case 2 -> backLeftTurnCanId;
              case 3 -> backRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    turnCAN =
        new CANcoder(
            switch (module) {
              case 0 -> frontLeftCANcoderID;
              case 1 -> frontRightCANcoderID;
              case 2 -> backLeftCANcoderID;
              case 3 -> backRightCANcoderID;
              default -> 0;
            });
    driveEncoder = driveSpark.getEncoder();
    driveController = driveSpark.getClosedLoopController();

    absPosSig = turnCAN.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, absPosSig);
    turnCAN.optimizeBusUtilization();

    // Configure drive motor
    var driveConfig = new SparkFlexConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(driveMotorCurrentLimit)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(driveEncoderPositionFactor)
        .velocityConversionFactor(driveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(driveKp, 0.0, driveKd);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

    var turnConfig = new SparkMaxConfig();
    switch (module) {
      case 0 -> turnConfig
          .inverted(!turnInverted)
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(turnMotorCurrentLimit)
          .voltageCompensation(12.0);
      case 1 -> turnConfig
          .inverted(!turnInverted)
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(turnMotorCurrentLimit)
          .voltageCompensation(12.0);
      case 2 -> turnConfig
          .inverted(!turnInverted)
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(turnMotorCurrentLimit)
          .voltageCompensation(12.0);
      case 3 -> turnConfig
          .inverted(!turnInverted)
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(turnMotorCurrentLimit)
          .voltageCompensation(12.0);
    }

    // turnConfig
    //     .absoluteEncoder
    //     .inverted(turnEncoderInverted)
    //     .positionConversionFactor(turnEncoderPositionFactor)
    //     .velocityConversionFactor(turnEncoderVelocityFactor)
    //     .averageDepth(2);
    // turnConfig
    //     .closedLoop
    //     .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    //     .positionWrappingEnabled(true)
    //     .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
    //     .pid(turnKp, 0.0, turnKd);
    turnConfig
        .signals
        // .absoluteEncoderPositionAlwaysOn(true)
        // .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        // .absoluteEncoderVelocityAlwaysOn(true)
        // .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    turnMotorPIDController.enableContinuousInput(0, 2 * 3.14159);

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    turnPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(turnSpark, this::getMeasuredAngleRad);
  }

  private double getMeasuredAngleRad() {
    absPosSig.refresh();
    double rot = absPosSig.getValueAsDouble();
    double rad = Units.rotationsToRadians(rot);
    return MathUtil.inputModulus(rad, 0.0, 2.0 * Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        turnSpark,
        this::getMeasuredAngleRad,
        (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
    ifOk(turnSpark, this::getMeasuredAngleRad, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
    // AutoLogOutputManager.addObject(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
    driveController.setSetpoint(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double maxOutput = 1.0;

    double meas = getMeasuredAngleRad();
    double goal =
        MathUtil.inputModulus(rotation.plus(zeroRotation).getRadians(), 0.0, 2.0 * Math.PI);
    double out = turnMotorPIDController.calculate(meas, goal);

    double capped = MathUtil.clamp(out, -maxOutput, maxOutput);
    turnSpark.set(-capped);

    var sp = turnMotorPIDController.getSetpoint();

    AutoLogOutputManager.addObject(sp);
  }
}
