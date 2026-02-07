package frc.robot.util.SingleMotorTests.SingleMotorVeocityPIDFTest;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import frc.robot.util.TunableControls.ControlConstants;
import frc.robot.util.TunableControls.TunableControlConstants;

public class SingleMotorVelocityPIDFTestConstants {
    // Motor Configs
    public static final double GEAR_RATIO = 1.0;

    // Gains (input: meters, output: volts)
    public static final ControlConstants CONTROL_CONSTANTS = new ControlConstants()
            .withPID(10, 0, 0)
            .withFeedforward(2.9, 0.0)
            .withPhysical(0.0, 0.2)
            .withProfile(2.0, 3.0)
            .withTolerance(0.04, 0.1)
            .withIZone(0.4);

    public static final TunableControlConstants TUNABLE_CONSTANTS =
            new TunableControlConstants("Elevator", CONTROL_CONSTANTS);
}