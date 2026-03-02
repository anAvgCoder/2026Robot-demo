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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.belt.BeltIntakeCommand;
import frc.robot.commands.belt.BeltOutakeCommand;
import frc.robot.commands.diverter.DiverterCommand;
import frc.robot.commands.intakepivot.IPIntakeCommand;
import frc.robot.commands.intakeroller.IRIntakeCommand;
import frc.robot.commands.intakeroller.IROutakeCommand;
import frc.robot.commands.shooter.ShooterSetSpeedCommand;
import frc.robot.subsystems.belt.Belt;
import frc.robot.subsystems.belt.BeltConstants;
import frc.robot.subsystems.belt.BeltIOReal;
import frc.robot.subsystems.belt.BeltIOSim;
import frc.robot.subsystems.diverter.Diverter;
import frc.robot.subsystems.diverter.DiverterIOReal;
import frc.robot.subsystems.diverter.DiverterIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakepivot.IntakePivotConstants;
import frc.robot.subsystems.intakepivot.IntakePivotIO;
import frc.robot.subsystems.intakepivot.IntakePivotIOReal;
import frc.robot.subsystems.intakeroller.IntakeRoller;
import frc.robot.subsystems.intakeroller.IntakeRollerIO;
import frc.robot.subsystems.intakeroller.IntakeRollerIOReal;
import frc.robot.subsystems.questnav.QuestNavSystem;
import frc.robot.subsystems.questnav.QuestNavSystemConstants;
import frc.robot.subsystems.questnav.QuestNavSystemIO;
import frc.robot.subsystems.questnav.QuestNavSystemIOReal;
import frc.robot.subsystems.turret.shooter.Shooter;
import frc.robot.subsystems.turret.shooter.ShooterIOReal;
import frc.robot.subsystems.turret.shooter.ShooterIOSim;
import frc.robot.util.SingleMotorTests.SingleMotorTurretPIDRioTest;
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
  private final Belt leftBelt;
  private final Belt rightBelt;
  private final IntakePivot intakePivot;
  private final IntakeRoller intakeRoller;
  private final Shooter leftShooter;
  private final Shooter rightShooter;
  private final Diverter diverter;
  private final SingleMotorTurretPIDRioTest singleMotorTurretPIDRioTest;

  // Commands

  // Joysticks
  private static final XboxController driverController = new XboxController(0);

  // Buttons
  private static final JoystickButton gyroButton = new JoystickButton(driverController, 4);
  private static final JoystickButton resetQuestPoseRedButton =
      new JoystickButton(driverController, 3);
  private static final JoystickButton resetQuestPoseBlueButton =
      new JoystickButton(driverController, 1);

  private static final JoystickButton intakeButton = new JoystickButton(driverController, 6);
  private static final JoystickButton outakeButton = new JoystickButton(driverController, 5);

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
                new QuestNavSystem(new QuestNavSystemIOReal()),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        diverter = new Diverter(new DiverterIOReal());
        leftBelt = new Belt(new BeltIOReal(BeltConstants.CanIdLeft));
        rightBelt = new Belt(new BeltIOReal(BeltConstants.CanIdRight));
        intakePivot = new IntakePivot(new IntakePivotIOReal(IntakePivotConstants.kCanId));
        intakeRoller = new IntakeRoller(new IntakeRollerIOReal());
        leftShooter = new Shooter(new ShooterIOReal(45));
        rightShooter = new Shooter(new ShooterIOReal(44));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new QuestNavSystem(new QuestNavSystemIO() {}),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        diverter = new Diverter(new DiverterIOSim());
        leftBelt = new Belt(new BeltIOSim());
        rightBelt = new Belt(new BeltIOSim());
        intakePivot = new IntakePivot(new IntakePivotIO() {});
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        leftShooter = new Shooter(new ShooterIOSim());
        rightShooter = new Shooter(new ShooterIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new QuestNavSystem(new QuestNavSystemIO() {}),
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        diverter = new Diverter(new DiverterIOSim());
        leftBelt = new Belt(new BeltIOSim() {});
        rightBelt = new Belt(new BeltIOSim() {});
        intakePivot = new IntakePivot(new IntakePivotIO() {});
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        leftShooter = new Shooter(new ShooterIOSim());
        rightShooter = new Shooter(new ShooterIOSim());
        break;
    }

    singleMotorTurretPIDRioTest = new SingleMotorTurretPIDRioTest();

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
            drive,
            () -> getClampedLeftX(driverController) ? -driverController.getLeftX() : 0.0,
            () -> getClampedY(driverController) ? driverController.getLeftY() : 0.0,
            () -> getClampedRightX(driverController) ? driverController.getRightX() : 0.0));

    intakeButton.whileTrue(
        Commands.parallel(
            new ShooterSetSpeedCommand(leftShooter),
            new ShooterSetSpeedCommand(rightShooter),
            Commands.sequence(
                Commands.waitSeconds(1.0),
                Commands.parallel(
                    new IRIntakeCommand(intakeRoller),
                    new BeltIntakeCommand(rightBelt),
                    new BeltIntakeCommand(leftBelt),
                    new DiverterCommand(diverter),
                    new IPIntakeCommand(intakePivot)))));

    outakeButton.whileTrue(
        Commands.parallel(
            new IROutakeCommand(intakeRoller),
            new BeltOutakeCommand(rightBelt),
            new BeltOutakeCommand(leftBelt),
            new DiverterCommand(diverter)));
    // new IPIntakeCommand(intakePivot)));

    // Reset gyro to 0 on press
    gyroButton.onTrue(
        Commands.runOnce(
                () ->
                    drive.setPose(
                        new Pose2d(
                            drive.getPose().getTranslation(),
                            Rotation2d.kZero.plus(new Rotation2d(-90)))))
            .ignoringDisable(true));

    resetQuestPoseRedButton.onTrue(
        Commands.runOnce(() -> drive.resetQuestPose(QuestNavSystemConstants.ROBOT_TO_QUEST_RED))
            .ignoringDisable(true));

    resetQuestPoseRedButton.onTrue(
        Commands.runOnce(() -> drive.resetQuestPose(QuestNavSystemConstants.ROBOT_TO_QUEST_BLUE))
            .ignoringDisable(true));
  }

  public boolean getClampedLeftX(XboxController joy) {
    return Math.abs(joy.getLeftX()) >= 0.1;
  }

  public boolean getClampedRightX(XboxController joy) {
    return Math.abs(joy.getRightX()) >= 0.1;
  }

  public boolean getClampedY(XboxController joy) {
    return Math.abs(joy.getLeftY()) >= 0.1;
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
