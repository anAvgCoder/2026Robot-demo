package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.belt.Belt;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Belt belt;
  private final Shooter shooter;

  // Operator interface
  private final Joystick leftStick = new Joystick(0);
  private final Joystick rightStick = new Joystick(1);
  private final Joystick buttonPanel = new Joystick(2);

  // Buttons
  private final JoystickButton button1 = new JoystickButton(buttonPanel, 1);
  private final JoystickButton button2 = new JoystickButton(buttonPanel, 2);
  private final JoystickButton button3 = new JoystickButton(buttonPanel, 3);
  private final JoystickButton button4 = new JoystickButton(buttonPanel, 4);
  private final JoystickButton button5 = new JoystickButton(buttonPanel, 5);
  private final JoystickButton button6 = new JoystickButton(buttonPanel, 6);
  private final JoystickButton button7 = new JoystickButton(buttonPanel, 7);
  private final JoystickButton button8 = new JoystickButton(buttonPanel, 8);
  private final JoystickButton button9 = new JoystickButton(buttonPanel, 9);
  private final JoystickButton button11 = new JoystickButton(buttonPanel, 11);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Subsystems subsystems = SubsystemFactory.create(Constants.currentMode);
    drive = subsystems.drive();
    intake = new Intake(10);
    belt = new Belt(30, 31);
    shooter = new Shooter(44);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> getClampedDrive(rightStick) ? -rightStick.getY() : 0.0,
            () -> getClampedDrive(rightStick) ? -rightStick.getX() : 0.0,
            () -> getClampedTurn(leftStick) ? -leftStick.getX() : 0.0));

    button1.onTrue(Commands.runOnce(() -> intake.suck()));
    button2.onTrue(Commands.runOnce(() -> intake.suckFast()));
    button3.onTrue(Commands.runOnce(() -> intake.suckSlow()));
    button4.onTrue(Commands.runOnce(() -> intake.vomit()));
    button5.onTrue(Commands.runOnce(() -> intake.off()));
    button6.onTrue(Commands.runOnce(() -> belt.convey()));
    button7.onTrue(Commands.runOnce(() -> belt.unconvey()));
    button8.onTrue(Commands.runOnce(() -> belt.stop()));
    button9.onTrue(Commands.runOnce(() -> shooter.spit(3500)));
    button11.onTrue(Commands.runOnce(() -> shooter.nope()));
  }

  public boolean getClampedTurn(Joystick joy) {
    return Math.abs(joy.getX()) >= 0.1;
  }

  public boolean getClampedDrive(Joystick joy) {
    return (Math.abs(joy.getY()) > 0.1) || (Math.abs(joy.getX()) > 0.1);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
