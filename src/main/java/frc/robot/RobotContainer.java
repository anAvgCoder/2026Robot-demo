// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkCANCoder;
import frc.robot.util.SingleMotorTests.SingleMotorVeocityPIDFTest.SingleMotorVelocityPIDFTest;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final SingleMotorVelocityPIDFTest singleMotorPIDFTest;

  // Controller
  // Joysticks
  private static final Joystick leftJoy = new Joystick(0);
  private static final Joystick rightJoy = new Joystick(1);
  private static final Joystick buttonPanel = new Joystick(2);

  // Buttons
  private final JoystickButton gyroButton = new JoystickButton(buttonPanel, 1);
  private final JoystickButton stopMotorButton = new JoystickButton(buttonPanel, 2);
  private final JoystickButton motor30Button = new JoystickButton(buttonPanel, 3);
  private final JoystickButton motor40Button = new JoystickButton(buttonPanel, 4);
  private final JoystickButton motor50Button = new JoystickButton(buttonPanel, 5);
  private final JoystickButton motor60Button = new JoystickButton(buttonPanel, 6);
  private final JoystickButton motor70Button = new JoystickButton(buttonPanel, 7);
  private final JoystickButton motor80Button = new JoystickButton(buttonPanel, 8);
  private final JoystickButton motor90 = new JoystickButton(buttonPanel, 9);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSparkCANCoder(0),
                new ModuleIOSparkCANCoder(1),
                new ModuleIOSparkCANCoder(2),
                new ModuleIOSparkCANCoder(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    singleMotorPIDFTest = new SingleMotorVelocityPIDFTest();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -rightJoy.getY(), () -> rightJoy.getX(), () -> -leftJoy.getX()));

    // Reset gyro to 0 on press
    gyroButton.onTrue(
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)))
            .ignoringDisable(true));

    stopMotorButton.onTrue(
        // Commands.runOnce(() -> singleMotorPIDFTest.setVelocity(0))
        Commands.run(() -> System.out.println(" iran ")));

    // motor30Button.onTrue(
    //   Commands.runOnce(() -> singleMotorPIDFTest.setVelocity(0.3)));

    // motor40Button.onTrue(
    //   Commands.runOnce(() -> singleMotorPIDFTest.setVelocity(0.4)));

    // motor50Button.onTrue(
    //   Commands.runOnce(() -> singleMotorPIDFTest.setVelocity(0.5)));

    // motor60Button.onTrue(
    //   Commands.runOnce(() -> singleMotorPIDFTest.setVelocity(0.6)));

    // motor70Button.onTrue(
    //   Commands.runOnce(() -> singleMotorPIDFTest.setVelocity(0.7)));

    // motor80Button.onTrue(
    //   Commands.runOnce(() -> singleMotorPIDFTest.setVelocity(0.8)));

    // motor90.onTrue(
    //   Commands.runOnce(() -> singleMotorPIDFTest.setVelocity(0.9)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
