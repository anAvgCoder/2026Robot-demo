// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.belt.BeltIntakeCommand;
import frc.robot.commands.belt.BeltOutakeCommand;
import frc.robot.commands.diverter.DiverterCommand;
import frc.robot.commands.intakepivot.IPIntakeCommand;
import frc.robot.commands.intakepivot.IPStorageCommand;
import frc.robot.commands.intakeroller.IRIntakeCommand;
import frc.robot.commands.intakeroller.IROutakeCommand;
import frc.robot.commands.shooter.ShooterAdaptiveHubAiming;
import frc.robot.commands.shooter.ShooterAdaptiveStorageAiming;
import frc.robot.commands.turret.AdaptiveHubAiming;
import frc.robot.commands.turret.AdaptiveHubNoMove;
import frc.robot.commands.turret.AdaptiveNoMove;
import frc.robot.commands.turret.AdaptiveStorageAiming;
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
import frc.robot.subsystems.turret.hood.Hood;
import frc.robot.subsystems.turret.hood.HoodConstants;
import frc.robot.subsystems.turret.hood.HoodIO;
import frc.robot.subsystems.turret.hood.HoodIOReal;
import frc.robot.subsystems.turret.rotater.Rotater;
import frc.robot.subsystems.turret.rotater.RotaterConstants;
import frc.robot.subsystems.turret.rotater.RotaterIO;
import frc.robot.subsystems.turret.rotater.RotaterIOReal;
import frc.robot.subsystems.turret.shooter.Shooter;
import frc.robot.subsystems.turret.shooter.ShooterIOReal;
import frc.robot.subsystems.turret.shooter.ShooterIOSim;
import java.util.Map;
import java.util.Set;
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
  private final Diverter diverter;

  private final Rotater leftRotater;
  private final Shooter leftShooter;
  private final Hood leftHood;

  private final Rotater rightRotater;
  private final Hood rightHood;
  private final Shooter rightShooter;

  // Commands

  // Joysticks
  private static final Joystick leftJoy = new Joystick(0);
  private static final Joystick rightJoy = new Joystick(1);
  private static final Joystick buttonPanel = new Joystick(2);

  // Buttons

  private static final JoystickButton intakeButton = new JoystickButton(rightJoy, 1);
  private static final JoystickButton FiftyPercentDriveButton = new JoystickButton(rightJoy, 3);

  private static final JoystickButton outakeButton = new JoystickButton(leftJoy, 1);
  private static final JoystickButton syncYawButton = new JoystickButton(leftJoy, 4);

  private static final JoystickButton panelButton1 = new JoystickButton(buttonPanel, 1);
  private static final JoystickButton panelButton2 = new JoystickButton(buttonPanel, 2);
  private static final JoystickButton panelButton3 = new JoystickButton(buttonPanel, 3);
  private static final JoystickButton panelButton4 = new JoystickButton(buttonPanel, 4);
  private static final JoystickButton panelButton5 = new JoystickButton(buttonPanel, 5);
  private static final JoystickButton panelButton6 = new JoystickButton(buttonPanel, 6);
  private static final JoystickButton panelButton7 = new JoystickButton(buttonPanel, 7);
  private static final JoystickButton panelButton8 = new JoystickButton(buttonPanel, 8);
  private static final JoystickButton panelButton9 = new JoystickButton(buttonPanel, 9);
  private static final JoystickButton panelButton11 = new JoystickButton(buttonPanel, 11);
  private static final JoystickButton panelButton12 = new JoystickButton(buttonPanel, 12);
  private static final JoystickButton panelButton13 = new JoystickButton(buttonPanel, 13);
  private static final JoystickButton panelButton14 = new JoystickButton(buttonPanel, 14);
  private static final JoystickButton panelButton15 = new JoystickButton(buttonPanel, 15);

  private static final JoystickButton testingButton1 = new JoystickButton(buttonPanel, 14);
  private static final JoystickButton testingButton2 = new JoystickButton(buttonPanel, 15);
  private static final JoystickButton testingButton3 = new JoystickButton(buttonPanel, 9);
  private static final JoystickButton testingButton4 = new JoystickButton(buttonPanel, 11);

  private static final JoystickButton resetQuestPoseRedButton = new JoystickButton(leftJoy, 11);
  private static final JoystickButton resetQuestPoseBlueButton = new JoystickButton(rightJoy, 11);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private int activePathToken = 0;

  private static PathPlannerPath loadPath(String name) {
    try {
      return PathPlannerPath.fromPathFile(name);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path: " + name, e.getStackTrace());
      throw new RuntimeException(e);
    }
  }

  private final Map<String, PathPlannerPath> paths =
      Map.of(
          "ST Push Balls", loadPath("ST Push Balls"),
          "ST Clear Depot", loadPath("ST Clear Depot"),
          "OP Push Balls", loadPath("OP Push Balls"));

  private Command activePathCommand = null;

  private final PathConstraints pathfindConstraints =
      new PathConstraints(1.5, 1.5, Units.degreesToRadians(540), Units.degreesToRadians(720));

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
        leftRotater =
            new Rotater(
                new RotaterIOReal(RotaterConstants.kCanIdLeft, RotaterConstants.kCanIdLeftCoder),
                "LeftRotater");
        leftHood = new Hood(new HoodIOReal(HoodConstants.kLeftCanId), "LeftHood");

        rightShooter = new Shooter(new ShooterIOReal(44));
        rightRotater =
            new Rotater(
                new RotaterIOReal(RotaterConstants.kCanIdRight, RotaterConstants.kCanIdRightCoder),
                "RightRotater");
        rightHood = new Hood(new HoodIOReal(HoodConstants.kRightCanId), "RightHood");

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
        leftRotater = new Rotater(new RotaterIO() {}, "LeftRotater");
        leftHood = new Hood(new HoodIO() {}, "LeftHood");

        rightShooter = new Shooter(new ShooterIOSim());
        rightRotater = new Rotater(new RotaterIO() {}, "RightRotater");
        rightHood = new Hood(new HoodIO() {}, "RightHood");
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
        leftRotater = new Rotater(new RotaterIO() {}, "LeftRotater");
        leftHood = new Hood(new HoodIO() {}, "LeftHood");

        rightShooter = new Shooter(new ShooterIOSim());
        rightRotater = new Rotater(new RotaterIO() {}, "RightRotater");
        rightHood = new Hood(new HoodIO() {}, "RightHood");
        break;
    }

    // Set up auto routines
    registerNamedCommands();

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
            () -> getClampedDrive(rightJoy) ? -rightJoy.getY() : 0.0,
            () -> getClampedDrive(rightJoy) ? -rightJoy.getX() : 0.0,
            () -> getClampedTurn(leftJoy) ? -leftJoy.getX() : 0.0));

    syncYawButton.toggleOnTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> getClampedDrive(rightJoy) ? -rightJoy.getY() : 0.0,
            () -> getClampedDrive(rightJoy) ? -rightJoy.getX() : 0.0,
            () ->
                MathUtil.angleModulus(
                    Math.PI
                        + Math.atan2(
                            getClampedDrive(rightJoy) ? -rightJoy.getX() : 0.0,
                            getClampedDrive(rightJoy) ? -rightJoy.getY() : 0.0))));

    FiftyPercentDriveButton.toggleOnTrue(
        DriveCommands.joystickDrive(
            drive,
            () -> getClampedDrive(rightJoy) ? -rightJoy.getY() * 0.5 : 0.0,
            () -> getClampedDrive(rightJoy) ? -rightJoy.getX() * 0.5 : 0.0,
            () -> getClampedTurn(leftJoy) ? -leftJoy.getX() * 0.5 : 0.0));

    intakeButton.whileTrue(
        Commands.parallel(
            new IRIntakeCommand(intakeRoller),
            Commands.sequence(
                Commands.waitSeconds(0.35),
                Commands.parallel(
                    new BeltIntakeCommand(rightBelt),
                    new BeltIntakeCommand(leftBelt),
                    new DiverterCommand(diverter),
                    new IPIntakeCommand(intakePivot)))));

    outakeButton.whileTrue(
        Commands.parallel(
            new IPIntakeCommand(intakePivot),
            Commands.sequence(
                Commands.waitSeconds(0.45),
                Commands.parallel(
                    new BeltOutakeCommand(rightBelt),
                    new BeltOutakeCommand(leftBelt),
                    new DiverterCommand(diverter),
                    new IROutakeCommand(intakeRoller)))));

    // adaptiveAimingButton.toggleOnTrue(
    //     new AdaptiveHubAimingOnlyTurret(rightRotater, leftRotater, drive, true));
    // adaptiveShooterTestAimingButton.toggleOnTrue(
    //     new TestHoodShooterCommand(rightShooter, rightHood, rightBelt));

    panelButton3
        .and(panelButton8.negate())
        .whileTrue(
            new AdaptiveHubNoMove(
                rightRotater,
                rightHood,
                leftRotater,
                leftHood,
                drive,
                (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Red)));

    panelButton2
        .and(panelButton8.negate())
        .whileTrue(
            new AdaptiveHubAiming(
                rightRotater,
                rightHood,
                leftRotater,
                leftHood,
                drive,
                (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Red)));

    panelButton1.toggleOnTrue(
        new ShooterAdaptiveHubAiming(
            rightShooter,
            leftShooter,
            drive,
            (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Red)));

    panelButton6
        .and(panelButton8.negate())
        .whileTrue(
            new AdaptiveNoMove(
                rightRotater,
                rightHood,
                leftRotater,
                leftHood,
                drive,
                (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Red)));

    panelButton5
        .and(panelButton8.negate())
        .whileTrue(
            new AdaptiveStorageAiming(
                rightRotater,
                rightHood,
                leftRotater,
                leftHood,
                drive,
                (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Red)));

    panelButton4.toggleOnTrue(
        new ShooterAdaptiveStorageAiming(
            rightShooter,
            leftShooter,
            drive,
            (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Red)));

    panelButton7.onTrue(cancelActivePath());
    panelButton8.whileTrue(forceHoodsDownAndIntake());

    resetQuestPoseRedButton.onTrue(
        Commands.runOnce(() -> drive.setPose(QuestNavSystemConstants.ROBOT_TO_QUEST_RED))
            .ignoringDisable(true));

    resetQuestPoseBlueButton.onTrue(
        Commands.runOnce(() -> drive.setPose(QuestNavSystemConstants.ROBOT_TO_QUEST_BLUE))
            .ignoringDisable(true));

    panelButton15.onTrue(
        Commands.runOnce(() -> drive.setPose(QuestNavSystemConstants.ROBOT_TO_QUEST_BLUE_TESTING))
            .ignoringDisable(true));

    panelButton9.onTrue(runTeleopPath("ST Push Balls"));
    panelButton11.onTrue(runTeleopPath("ST Clear Depot"));
    panelButton12.onTrue(runTeleopPath("OP Push Balls"));
    panelButton13.onTrue(runTeleopPath("OP Clear Depot"));
  }

  public boolean getClampedTurn(Joystick joy) {
    return Math.abs(joy.getX()) >= 0.1;
  }

  public boolean getClampedDrive(Joystick joy) {
    return (Math.abs(joy.getY()) > 0.1) || (Math.abs(joy.getX()) > 0.1);
  }

  private Command runTeleopPath(String name) {
    PathPlannerPath path = paths.get(name);
    if (path == null) {
      return Commands.runOnce(() -> DriverStation.reportError("Unknown path name: " + name, false));
    }

    return Commands.defer(
        () -> {
          cancelActivePathNow();

          final int myToken = ++activePathToken;

          Command cmd =
              AutoBuilder.pathfindThenFollowPath(path, pathfindConstraints)
                  .withName("TeleopPath_" + name)
                  .finallyDo(
                      interrupted -> {
                        if (activePathToken == myToken) {
                          activePathCommand = null;
                        }
                      });

          activePathCommand = cmd;
          return cmd;
        },
        Set.of(drive));
  }

  private Command cancelActivePath() {
    return Commands.runOnce(this::cancelActivePathNow);
  }

  private void cancelActivePathNow() {
    if (activePathCommand != null) {
      CommandScheduler.getInstance().cancel(activePathCommand);
      activePathCommand = null;
    }
  }

  private void registerNamedCommands() {

    System.out.println("Commands registered");

    NamedCommands.registerCommand(
        "StartPick",
        Commands.parallel(
            Commands.runOnce(() -> intakeRoller.getIO().intake()),
            Commands.sequence(
                Commands.waitSeconds(0.35),
                Commands.runOnce(() -> intakePivot.getIO().setIntakeSecondaryPosition()),
                Commands.runOnce(() -> leftBelt.getIO().intake()),
                Commands.runOnce(() -> rightBelt.getIO().intake()),
                Commands.runOnce(() -> diverter.getIO().intake()))));

    NamedCommands.registerCommand(
        "StopPick",
        Commands.parallel(
            Commands.runOnce(() -> intakeRoller.getIO().stop()),
            Commands.sequence(
                Commands.waitSeconds(0.35),
                Commands.runOnce(() -> intakePivot.getIO().setStoragePosition()),
                Commands.runOnce(() -> leftBelt.getIO().stop()),
                Commands.runOnce(() -> rightBelt.getIO().stop()),
                Commands.runOnce(() -> diverter.getIO().stop()))));

    NamedCommands.registerCommand(
        "AdaptiveStorageAimingShooter",
        Commands.repeatingSequence(
            new ShooterAdaptiveStorageAiming(
                rightShooter,
                leftShooter,
                drive,
                (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Red))));

    NamedCommands.registerCommand(
        "AdaptiveStorageAimingNoMove",
        Commands.repeatingSequence(
            new AdaptiveNoMove(
                rightRotater,
                rightHood,
                leftRotater,
                leftHood,
                drive,
                (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Red))));
  }

  private Command forceHoodsDownAndIntake() {
    Command holdHoodsDown =
        Commands.run(
            () -> {
              rightHood.getIO().setHoodPosition(0.0);
              leftHood.getIO().setHoodPosition(0.0);
            },
            rightHood,
            leftHood);

    Command intake = Commands.parallel(new IPStorageCommand(intakePivot));

    return holdHoodsDown.alongWith(intake).withName("ForceHoodsDownAndIntake");
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
