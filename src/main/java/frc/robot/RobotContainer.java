package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakePivot.IntakePivotConstants;
import frc.robot.subsystems.intakePivot.IntakePivotIO;
import frc.robot.subsystems.intakePivot.IntakePivotIOReal;
import frc.robot.subsystems.intakePivot.IntakePivotIOSim;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.intakeRoller.IntakeRollerIO;
import frc.robot.subsystems.intakeRoller.IntakeRollerIOReal;
import frc.robot.subsystems.intakeRoller.IntakeRollerIOSim;
import frc.robot.subsystems.questNav.QuestNavConstants;
import frc.robot.subsystems.questNav.QuestNavSensor;
import frc.robot.subsystems.turret.ShotTable;
import frc.robot.subsystems.turret.hood.Hood;
import frc.robot.subsystems.turret.hood.HoodConstants;
import frc.robot.subsystems.turret.hood.HoodIO;
import frc.robot.subsystems.turret.hood.HoodIOReal;
import frc.robot.subsystems.turret.hood.HoodIOSim;
import frc.robot.subsystems.turret.rotater.Rotater;
import frc.robot.subsystems.turret.rotater.RotaterConstants;
import frc.robot.subsystems.turret.rotater.RotaterIO;
import frc.robot.subsystems.turret.rotater.RotaterIOReal;
import frc.robot.subsystems.turret.rotater.RotaterIOSim;
import frc.robot.subsystems.turret.shooter.Shooter;
import frc.robot.subsystems.turret.shooter.ShooterIOReal;
import frc.robot.subsystems.turret.shooter.ShooterIOSim;
import java.util.Map;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

  private final ShotTable shotTable;

  // Joysticks
  private static final Joystick leftJoy = new Joystick(0);
  private static final Joystick rightJoy = new Joystick(1);
  private static final Joystick buttonPanel = new Joystick(2);

  // Buttons

  private static final JoystickButton rightJoy5Button = new JoystickButton(rightJoy, 5);
  private static final JoystickButton rightJoy6Button = new JoystickButton(rightJoy, 6);
  private static final JoystickButton rightJoy7Button = new JoystickButton(rightJoy, 7);
  private static final JoystickButton rightJoy8Button = new JoystickButton(rightJoy, 8);
  private static final JoystickButton rightJoy9Button = new JoystickButton(rightJoy, 9);
  private static final JoystickButton rightJoy10Button = new JoystickButton(rightJoy, 10);

  private static final JoystickButton intakeButton = new JoystickButton(rightJoy, 1);
  private static final JoystickButton deployOutakeButton = new JoystickButton(rightJoy, 2);
  private static final JoystickButton fiftyPercentDriveButton = new JoystickButton(rightJoy, 3);

  private static final JoystickButton leftJoy3Button = new JoystickButton(leftJoy, 3);
  private static final JoystickButton rightJoy4Button = new JoystickButton(rightJoy, 4);

  private static final JoystickButton outakeButton = new JoystickButton(leftJoy, 1);
  private static final JoystickButton retractOutakeButton = new JoystickButton(leftJoy, 2);
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

  private static final JoystickButton resetQuestPoseRedButton = new JoystickButton(leftJoy, 11);
  private static final JoystickButton resetQuestPoseBlueButton = new JoystickButton(rightJoy, 11);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private int activePathToken = 0;
  private Command activePathCommand = null;

  private static final Map<String, String> TELEOP_PATH_FILES =
      Map.of(
          "ST Push Balls", "ST Push Balls",
          "ST Clear Depot", "ST Clear Depot",
          "OP Push Balls", "OP Push Balls",
          "OP Clear Depot", "OP Clear Depot");

  private final PathConstraints pathfindConstraints =
      new PathConstraints(1.5, 1.5, Units.degreesToRadians(540), Units.degreesToRadians(720));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new QuestNavSensor(),
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
        drive =
            new Drive(
                new GyroIO() {},
                new QuestNavSensor(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        diverter = new Diverter(new DiverterIOSim());
        leftBelt = new Belt(new BeltIOSim());
        rightBelt = new Belt(new BeltIOSim());
        intakePivot = new IntakePivot(new IntakePivotIOSim());
        intakeRoller = new IntakeRoller(new IntakeRollerIOSim());

        leftShooter = new Shooter(new ShooterIOSim());
        leftRotater = new Rotater(new RotaterIOSim(), "LeftRotater");
        leftHood = new Hood(new HoodIOSim(), "LeftHood");

        rightShooter = new Shooter(new ShooterIOSim());
        rightRotater = new Rotater(new RotaterIOSim(), "RightRotater");
        rightHood = new Hood(new HoodIOSim(), "RightHood");
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new QuestNavSensor(),
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

    shotTable = new ShotTable();

    // Must happen before any PathPlanner path/auto is constructed.
    registerNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureButtonBindings();
  }

  /**
   * Use this method to define button->command mappings.
   *
   * <p>Hold-style controls use whileTrue so the command is canceled automatically on release.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> getClampedDrive(rightJoy) ? -rightJoy.getY() * 0.8 : 0.0,
            () -> getClampedDrive(rightJoy) ? -rightJoy.getX() * 0.8 : 0.0,
            () -> getClampedTurn(leftJoy) ? -leftJoy.getX() * 0.8 : 0.0));

    // syncYawButton.whileTrue(
    //     DriveCommands.joystickDriveAtAngle(
    //         drive,
    //         () -> getClampedDrive(rightJoy) ? -rightJoy.getY() * 0.8 : 0.0,
    //         () -> getClampedDrive(rightJoy) ? -rightJoy.getX() * 0.8 : 0.0,
    //         () ->
    //             MathUtil.angleModulus(
    //                 Math.PI
    //                     + Math.atan2(
    //                         getClampedDrive(rightJoy) ? -rightJoy.getX() * 0.8 : 0.0,
    //                         getClampedDrive(rightJoy) ? -rightJoy.getY() * 0.8 : 0.0))));

    // fiftyPercentDriveButton.whileTrue(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> getClampedDrive(rightJoy) ? -rightJoy.getY() * 0.5 : 0.0,
    //         () -> getClampedDrive(rightJoy) ? -rightJoy.getX() * 0.5 : 0.0,
    //         () -> getClampedTurn(leftJoy) ? -leftJoy.getX() * 0.5 : 0.0));

    intakeButton.whileTrue(
        Commands.parallel(
            new IPIntakeCommand(intakePivot),
            new IRIntakeCommand(intakeRoller),
            new BeltIntakeCommand(rightBelt),
            new BeltIntakeCommand(leftBelt),
            new DiverterCommand(diverter)));

    outakeButton.whileTrue(
        Commands.parallel(
            new BeltOutakeCommand(rightBelt),
            new BeltOutakeCommand(leftBelt),
            new DiverterCommand(diverter),
            new IROutakeCommand(intakeRoller)));

    deployOutakeButton.onTrue(new IPIntakeCommand(intakePivot));
    retractOutakeButton.onTrue(new IPStorageCommand(intakePivot));

    panelButton14.onTrue(
        Commands.runOnce(
            () -> {
              leftRotater.setTurnPosition(0);
              rightRotater.setTurnPosition(0);
            }));
    panelButton13.onTrue(
        Commands.runOnce(
            () -> {
              leftRotater.setTurnPosition(-60);
              rightRotater.setTurnPosition(-60);
            }));
    panelButton11.onTrue(
        Commands.runOnce(
            () -> {
              leftRotater.setTurnPosition(-90);
              rightRotater.setTurnPosition(-90);
            }));
    panelButton12.onTrue(
        Commands.runOnce(
            () -> {
              leftRotater.setTurnPosition(60);
              rightRotater.setTurnPosition(60);
            }));
    panelButton9.onTrue(
        Commands.runOnce(
            () -> {
              leftRotater.setTurnPosition(90);
              rightRotater.setTurnPosition(90);
            }));

    // panelButton3.and(panelButton8.negate()).whileTrue(adaptiveHubNoMoveCommand());
    // panelButton6.and(panelButton8.negate()).whileTrue(adaptiveStorageNoMoveCommand());

    // panelButton1.toggleOnTrue(shooterAdaptiveHubAimingCommand());
    // panelButton4.toggleOnTrue(shooterAdaptiveStorageAimingCommand());

    // panelButton5.and(panelButton8.negate()).whileTrue(adaptiveStorageAimingCommand());
    // panelButton2.and(panelButton8.negate()).whileTrue(adaptiveHubAimingCommand());

    // panelButton7.onTrue(cancelActivePath());

    panelButton1.toggleOnTrue(
        Commands.runOnce(
            () -> {
              leftShooter.setSpeed(3600);
              rightShooter.setSpeed(3600);
              leftHood.setHoodPosition(0.3);
              rightHood.setHoodPosition(0.3);
            }));
    panelButton2.toggleOnTrue(
        Commands.runOnce(
            () -> {
              leftShooter.setSpeed(3300);
              rightShooter.setSpeed(3300);
              leftHood.setHoodPosition(0.3);
              rightHood.setHoodPosition(0.3);
            }));
    panelButton3.toggleOnTrue(
        Commands.runOnce(
            () -> {
              leftShooter.setSpeed(3000);
              rightShooter.setSpeed(3000);
              leftHood.setHoodPosition(0.3);
              rightHood.setHoodPosition(0.3);
            }));
    panelButton5.toggleOnTrue(
        Commands.runOnce(
            () -> {
              leftShooter.setSpeed(4200);
              rightShooter.setSpeed(4200);
              leftHood.setHoodPosition(0.3);
              rightHood.setHoodPosition(0.3);
            }));
    panelButton4.toggleOnTrue(
        Commands.runOnce(
            () -> {
              leftShooter.stop();
              rightShooter.stop();
              leftHood.setHoodPosition(0.3);
              rightHood.setHoodPosition(0.3);
            }));
    panelButton6.toggleOnTrue(
        Commands.runOnce(
            () -> {
              leftShooter.setSpeed(3900);
              rightShooter.setSpeed(3900);
              leftHood.setHoodPosition(0.3);
              rightHood.setHoodPosition(0.3);
            }));

    panelButton15.onTrue(
        Commands.runOnce(
            () -> {
              shotTable.setMultiFactor(shotTable.getMultiFactor() + 0.05);
            }));
    panelButton7.onTrue(
        Commands.runOnce(
            () -> {
              shotTable.setMultiFactor(shotTable.getMultiFactor() - 0.05);
            }));

    panelButton8.onTrue(
        Commands.parallel(
            new IPStorageCommand(intakePivot),
            Commands.runOnce(() -> leftHood.setVoltage(-1.0))
                .andThen(Commands.waitSeconds(0.25))
                .andThen(
                    Commands.runOnce(
                        () -> {
                          leftHood.zeroAtMin();
                          leftHood.setVoltage(0.0);
                        })),
            Commands.runOnce(() -> rightHood.setVoltage(-1.0))
                .andThen(Commands.waitSeconds(0.25))
                .andThen(
                    Commands.runOnce(
                        () -> {
                          rightHood.zeroAtMin();
                          rightHood.setVoltage(0.0);
                        }))));

    resetQuestPoseRedButton.onTrue(
        Commands.runOnce(() -> drive.setPose(QuestNavConstants.ROBOT_TO_QUEST_RED))
            .ignoringDisable(true));

    resetQuestPoseBlueButton.onTrue(
        Commands.runOnce(() -> drive.setPose(QuestNavConstants.ROBOT_TO_QUEST_BLUE))
            .ignoringDisable(true));

    // panelButton15.onTrue(
    //     Commands.runOnce(() ->
    // drive.setPose(QuestNavSystemConstants.ROBOT_TO_QUEST_BLUE_TESTING))
    //         .ignoringDisable(true));
    // leftJoy3Button.onTrue(
    //     Commands.runOnce(() ->
    // drive.setPose(QuestNavSystemConstants.ROBOT_TO_QUEST_BLUE_TESTING))
    //         .ignoringDisable(true));

    // leftJoy3Button.onTrue(runTeleopPath("ST Push Balls"));
    // panelButton11.onTrue(runTeleopPath("ST Clear Depot"));
    // rightJoy4Button.onTrue(runTeleopPath("OP Push Balls"));
    // panelButton13.onTrue(runTeleopPath("OP Clear Depot"));
  }

  public boolean getClampedTurn(Joystick joy) {
    return Math.abs(joy.getX()) >= 0.1;
  }

  public boolean getClampedDrive(Joystick joy) {
    return (Math.abs(joy.getY()) > 0.1) || (Math.abs(joy.getX()) > 0.1);
  }

  private boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Red;
  }

  private Command adaptiveHubNoMoveCommand() {
    return deferredCommand(
        () ->
            new AdaptiveHubNoMove(
                rightRotater, rightHood, leftRotater, leftHood, drive, isBlueAlliance(), shotTable),
        rightRotater,
        rightHood,
        leftRotater,
        leftHood);
  }

  private Command adaptiveHubAimingCommand() {
    return deferredCommand(
        () ->
            new AdaptiveHubAiming(
                rightRotater, rightHood, leftRotater, leftHood, drive, isBlueAlliance(), shotTable),
        rightRotater,
        rightHood,
        leftRotater,
        leftHood);
  }

  private Command adaptiveStorageNoMoveCommand() {
    return deferredCommand(
        () ->
            new AdaptiveNoMove(
                rightRotater, rightHood, leftRotater, leftHood, drive, isBlueAlliance(), shotTable),
        rightRotater,
        rightHood,
        leftRotater,
        leftHood);
  }

  private Command adaptiveStorageAimingCommand() {
    return deferredCommand(
        () ->
            new AdaptiveStorageAiming(
                rightRotater, rightHood, leftRotater, leftHood, drive, isBlueAlliance(), shotTable),
        rightRotater,
        rightHood,
        leftRotater,
        leftHood);
  }

  private Command shooterAdaptiveHubAimingCommand() {
    return deferredCommand(
        () ->
            new ShooterAdaptiveHubAiming(
                rightShooter, leftShooter, drive, isBlueAlliance(), shotTable),
        rightShooter,
        leftShooter);
  }

  private Command shooterAdaptiveStorageAimingCommand() {
    return deferredCommand(
        () ->
            new ShooterAdaptiveStorageAiming(
                rightShooter, leftShooter, drive, isBlueAlliance(), shotTable),
        rightShooter,
        leftShooter);
  }

  private Command runTeleopPath(String name) {
    String pathFile = TELEOP_PATH_FILES.get(name);
    if (pathFile == null) {
      return Commands.runOnce(() -> DriverStation.reportError("Unknown path name: " + name, false));
    }

    return Commands.defer(
        () -> {
          cancelActivePathNow();

          final PathPlannerPath path;
          try {
            path = PathPlannerPath.fromPathFile(pathFile);
          } catch (Exception e) {
            DriverStation.reportError("Failed to load path: " + pathFile, e.getStackTrace());
            return Commands.none();
          }

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
    NamedCommands.registerCommand(
        "StartPick",
        Commands.runOnce(
            () -> {
              intakePivot.setIntakeSecondaryPosition();
              intakeRoller.getIO().intake();
              leftBelt.getIO().intake();
              rightBelt.getIO().intake();
              diverter.getIO().intake();
            }));

    NamedCommands.registerCommand(
        "AdaptiveStorageAimingShooter", shooterAdaptiveStorageAimingCommand());

    NamedCommands.registerCommand("AdaptiveStorageAimingNoMove", adaptiveStorageNoMoveCommand());
  }

  private Command deferredCommand(
      java.util.function.Supplier<Command> supplier, Subsystem... requirements) {
    return Commands.defer(supplier, Set.of(requirements));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
    // return null;
  }
}
