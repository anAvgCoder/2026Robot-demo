package frc.robot;

import static frc.robot.subsystems.questnav.QuestNavConstants.ROBOT_TO_QUEST;
import static frc.robot.subsystems.questnav.QuestNavConstants.ROBOT_TO_QUEST_RED_HUB;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.belt.BeltIntakeCommand;
import frc.robot.commands.belt.BeltOutakeCommand;
import frc.robot.commands.diverter.DiverterCommand;
import frc.robot.commands.intakepivot.IPIntakeCommand;
import frc.robot.commands.intakepivot.IPStorageCommand;
import frc.robot.commands.intakeroller.IRIntakeCommand;
import frc.robot.commands.intakeroller.IROutakeCommand;
import frc.robot.commands.turret.AdaptiveHubAiming;
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
import frc.robot.subsystems.intakepivot.IntakePivotIOReal;
import frc.robot.subsystems.intakepivot.IntakePivotIOReplay;
import frc.robot.subsystems.intakepivot.IntakePivotIOSim;
import frc.robot.subsystems.intakeroller.IntakeRoller;
import frc.robot.subsystems.intakeroller.IntakeRollerIO;
import frc.robot.subsystems.intakeroller.IntakeRollerIOReal;
import frc.robot.subsystems.intakeroller.IntakeRollerIOSim;
import frc.robot.subsystems.questnav.QuestNavConstants;
import frc.robot.subsystems.questnav.QuestNavSensor;
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
import frc.robot.subsystems.vision.PhotonVisionIO;
import frc.robot.subsystems.vision.PhotonVisionSimIO;
import frc.robot.subsystems.vision.Vision;
import java.util.Map;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final QuestNavSensor questSensor;
  private final Vision vision;
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
  private static final JoystickButton rightJoy1Button = new JoystickButton(rightJoy, 1);
  private static final JoystickButton rightJoy2Button = new JoystickButton(rightJoy, 2);
  private static final JoystickButton rightJoy3Button = new JoystickButton(rightJoy, 3);
  private static final JoystickButton rightJoy4Button = new JoystickButton(rightJoy, 4);
  private static final JoystickButton rightJoy5Button = new JoystickButton(rightJoy, 5);
  private static final JoystickButton rightJoy6Button = new JoystickButton(rightJoy, 6);
  private static final JoystickButton rightJoy7Button = new JoystickButton(rightJoy, 7);
  private static final JoystickButton rightJoy8Button = new JoystickButton(rightJoy, 8);
  private static final JoystickButton rightJoy9Button = new JoystickButton(rightJoy, 9);
  private static final JoystickButton rightJoy10Button = new JoystickButton(rightJoy, 10);

  private static final JoystickButton outakeButton = new JoystickButton(leftJoy, 1);
  private static final JoystickButton leftJoy2Button = new JoystickButton(leftJoy, 2);
  private static final JoystickButton leftJoy3Button = new JoystickButton(leftJoy, 3);
  private static final JoystickButton leftJoy4Button = new JoystickButton(leftJoy, 4);

  private static final JoystickButton leftJoy14Button = new JoystickButton(leftJoy, 14);
  private static final JoystickButton leftJoy13Button = new JoystickButton(leftJoy, 13);

  private static final JoystickButton resetQuestPoseRedButton = new JoystickButton(leftJoy, 11);
  private static final JoystickButton resetQuestPoseRedHubButton = new JoystickButton(leftJoy, 5);

  private static final JoystickButton resetQuestPoseBlueButton = new JoystickButton(rightJoy, 11);
  private static final JoystickButton resetQuestPoseBlueHubButton = new JoystickButton(rightJoy, 5);

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

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Command> turretControlChooser;
  private int activePathToken = 0;
  private Command activePathCommand = null;

  private Command activeAutoIntakeCommand = null;
  private Command activeAdaptiveHubAimingCommand = null;

  private static final Map<String, String> TELEOP_PATH_FILES =
      Map.of(
          "ST Push Balls", "ST Push Balls",
          "ST Clear Depot", "ST Clear Depot",
          "OP Push Balls", "OP Push Balls",
          "OP Clear Depot", "OP Clear Depot");

  private final PathConstraints pathfindConstraints =
      new PathConstraints(1.5, 1.5, Units.degreesToRadians(540), Units.degreesToRadians(720));

  private boolean isPaused = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    questSensor = new QuestNavSensor();

    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                questSensor,
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        vision = new Vision(new PhotonVisionIO(drive));

        diverter = new Diverter(new DiverterIOReal());

        leftBelt = new Belt(new BeltIOReal(BeltConstants.CanIdLeft), "BeltLeft");

        rightBelt = new Belt(new BeltIOReal(BeltConstants.CanIdRight), "BeltRight");
        intakePivot =
            new IntakePivot(
                new IntakePivotIOReal(
                    IntakePivotConstants.kCanId, IntakePivotConstants.kSwitchDIOChannel));
        intakeRoller = new IntakeRoller(new IntakeRollerIOReal());

        leftShooter = new Shooter(new ShooterIOReal(45), "ShooterLeft");
        leftRotater =
            new Rotater(
                new RotaterIOReal(RotaterConstants.kCanIdLeft, RotaterConstants.kCanIdLeftCoder),
                "RotaterLeft");
        leftHood = new Hood(new HoodIOReal(HoodConstants.kLeftCanId), "HoodLeft");

        rightShooter = new Shooter(new ShooterIOReal(44), "ShooterRight");
        rightRotater =
            new Rotater(
                new RotaterIOReal(RotaterConstants.kCanIdRight, RotaterConstants.kCanIdRightCoder),
                "RotaterRight");
        rightHood = new Hood(new HoodIOReal(HoodConstants.kRightCanId), "HoodRight");
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                questSensor,
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision = new Vision(new PhotonVisionSimIO());
        diverter = new Diverter(new DiverterIOSim());
        leftBelt = new Belt(new BeltIOSim(), "BeltLeft");
        rightBelt = new Belt(new BeltIOSim(), "BeltRight");
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
                questSensor,
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(new PhotonVisionSimIO());

        diverter = new Diverter(new DiverterIOSim());
        leftBelt = new Belt(new BeltIOSim() {}, "BeltLeft");
        rightBelt = new Belt(new BeltIOSim() {}, "BeltRIght");
        intakePivot = new IntakePivot(new IntakePivotIOReplay());
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

    turretControlChooser = new LoggedDashboardChooser<>("Turret Control Type");
    turretControlChooser.addDefaultOption("Automatic", adaptiveHubAimingCommand());
    turretControlChooser.addOption(
        "Manual", Commands.runOnce(() -> adaptiveHubAimingCommand().cancel()));

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> getClampedDrive(rightJoy) ? -rightJoy.getY() : 0.0,
            () -> getClampedDrive(rightJoy) ? -rightJoy.getX() : 0.0,
            () -> getClampedTurn(leftJoy) ? -leftJoy.getX() : 0.0));

    rightJoy1Button.whileTrue(intakePickupHeldCommand());
    rightJoy1Button.onFalse(intakePickupReleaseCommand());

    rightJoy2Button.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> getClampedDrive(rightJoy) ? -rightJoy.getY() : 0.0,
            () -> getClampedDrive(rightJoy) ? -rightJoy.getX() : 0.0));

    rightJoy3Button.whileTrue(Commands.runOnce(drive::setSpeedFull).withName("set speed full"));
    rightJoy3Button.onFalse(Commands.runOnce(drive::setSpeedNormal).withName("set speed normal"));

    rightJoy4Button.whileTrue(Commands.runOnce(drive::setSpeedIntake).withName("setSpeedIntake"));
    rightJoy4Button.onFalse(Commands.runOnce(drive::setSpeedNormal).withName("setSpeedNormal"));

    outakeButton.whileTrue(
        Commands.parallel(
                new BeltOutakeCommand(rightBelt),
                new BeltOutakeCommand(leftBelt),
                new DiverterCommand(diverter),
                new IROutakeCommand(intakeRoller))
            .withName("Run Outtake"));

    leftJoy2Button.onTrue(new IPStorageCommand(intakePivot));
    leftJoy2Button.onFalse(new IPIntakeCommand(intakePivot));

    panelButton2.onTrue(
        Commands.runOnce(
                () -> {
                  leftRotater.setTurnPosition(0);
                  rightRotater.setTurnPosition(0);
                })
            .withName("RotateTo:0"));
    panelButton3.onTrue(
        Commands.runOnce(
                () -> {
                  leftRotater.setTurnPosition(-60);
                  rightRotater.setTurnPosition(-60);
                })
            .withName("RotateTo:-60"));
    panelButton6.onTrue(
        Commands.runOnce(
                () -> {
                  leftRotater.setTurnPosition(-90);
                  rightRotater.setTurnPosition(-90);
                })
            .withName("RotateTo:-90"));
    panelButton1.onTrue(
        Commands.runOnce(
                () -> {
                  leftRotater.setTurnPosition(60);
                  rightRotater.setTurnPosition(60);
                })
            .withName("RotateTo:60"));
    panelButton4.onTrue(
        Commands.runOnce(
                () -> {
                  leftRotater.setTurnPosition(90);
                  rightRotater.setTurnPosition(90);
                })
            .withName("RotateTo:90"));

    panelButton5.onTrue(
        Commands.runOnce(
                () -> {
                  isPaused = true;
                  leftHood.pause();
                  rightHood.pause();
                  rightBelt.pause();
                  leftBelt.pause();
                  diverter.pause();
                  intakeRoller.pause();
                })
            .withName("Pause all"));

    panelButton5.onFalse(
        Commands.runOnce(
                () -> {
                  isPaused = false;
                  leftHood.resume();
                  rightHood.resume();
                  rightBelt.resume();
                  leftBelt.resume();
                  diverter.resume();
                  intakeRoller.resume();
                })
            .withName("Resume all"));

    panelButton7.onTrue(
        Commands.runOnce(
                () -> {
                  drive.switchToCamera();
                })
            .ignoringDisable(true));

    panelButton8.onTrue(
        Commands.runOnce(
                () -> {
                  drive.switchToQuest();
                })
            .ignoringDisable(true));

    panelButton9.onTrue(
        Commands.runOnce(
            () -> {
              leftHood.addAngleDeg(-Constants.manualHoodIncDegrees);
              rightHood.addAngleDeg(-Constants.manualHoodIncDegrees);
            }));

    panelButton11.onTrue(
        Commands.runOnce(
            () -> {
              leftHood.addAngleDeg(Constants.manualHoodIncDegrees);
              rightHood.addAngleDeg(Constants.manualHoodIncDegrees);
            }));

    panelButton12.onTrue(
        Commands.runOnce(
            () -> {
              leftRotater.addTurnPosition(-Constants.manualHoodIncDegrees);
              rightRotater.addTurnPosition(-Constants.manualHoodIncDegrees);
            }));

    panelButton13.onTrue(
        Commands.runOnce(
            () -> {
              leftRotater.addTurnPosition(Constants.manualRotationIncDegrees);
              rightRotater.addTurnPosition(Constants.manualRotationIncDegrees);
            }));

    panelButton14.onTrue(
        Commands.runOnce(
            () -> {
              leftShooter.addVelocityRPM(-Constants.manualShooterIncRPM);
              rightShooter.addVelocityRPM(-Constants.manualShooterIncRPM);
            }));

    panelButton15.onTrue(
        Commands.runOnce(
            () -> {
              leftShooter.addVelocityRPM(Constants.manualShooterIncRPM);
              rightShooter.addVelocityRPM(Constants.manualShooterIncRPM);
            }));

    resetQuestPoseRedButton.onTrue(
        Commands.runOnce(
                () ->
                    drive.setPose(
                        QuestNavConstants.ROBOT_TO_QUEST_RED.transformBy(ROBOT_TO_QUEST.inverse())))
            .ignoringDisable(true)
            .withName("ResetPoseRed"));

    resetQuestPoseBlueButton.onTrue(
        Commands.runOnce(
                () ->
                    drive.setPose(
                        QuestNavConstants.ROBOT_TO_QUEST_BLUE.transformBy(
                            ROBOT_TO_QUEST.inverse())))
            .ignoringDisable(true)
            .withName("ResetPoseBlue"));

    resetQuestPoseBlueHubButton.onTrue(
        Commands.runOnce(
                () ->
                    drive.setPose(
                        QuestNavConstants.ROBOT_TO_QUEST_BLUE_HUB.transformBy(
                            ROBOT_TO_QUEST.inverse())))
            .ignoringDisable(true)
            .withName("ResetPoseBlue"));

    resetQuestPoseRedHubButton.onTrue(
        Commands.runOnce(
                () -> drive.setPose(ROBOT_TO_QUEST_RED_HUB.transformBy(ROBOT_TO_QUEST.inverse())))
            .ignoringDisable(true)
            .withName("ResetPoseBlue"));
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

  private Command intakePickupHeldCommand() {
    return Commands.parallel(
            Commands.runOnce(
                () -> {
                  drive.setSpeedIntake();
                  if (!isPaused) {
                    leftHood.resume();
                    rightHood.resume();
                  }
                }),
            new IPIntakeCommand(intakePivot),
            new IRIntakeCommand(intakeRoller),
            new BeltIntakeCommand(rightBelt),
            new BeltIntakeCommand(leftBelt),
            new DiverterCommand(diverter))
        .withName("Run Intake");
  }

  private Command runRollers() {
    return Commands.parallel(new IRIntakeCommand(intakeRoller).withName("Run Intake"));
  }

  private Command intakePickupOutCommand() {
    return Commands.parallel(new IPIntakeCommand(intakePivot).withName("Run IntakePivot Out"));
  }

  private Command intakePickupReleaseCommand() {
    return Commands.runOnce(
            () -> {
              drive.setSpeedNormal();
              leftHood.pause();
              rightHood.pause();
            })
        .withName("Stop Intake");
  }

  private Command stopDrive() {
    return Commands.runOnce(
            () -> {
              drive.stop();
            })
        .withName("Stop Drive");
  }

  private Command adaptiveHubAimingCommand() {
    return deferredCommand(
        () ->
            new AdaptiveHubAiming(
                rightShooter,
                rightRotater,
                rightHood,
                leftShooter,
                leftRotater,
                leftHood,
                drive,
                isBlueAlliance(),
                shotTable),
        rightRotater,
        rightHood,
        leftRotater,
        leftHood);
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

  private Command ejectFuelCommand() {
    return Commands.parallel(
            new BeltOutakeCommand(rightBelt),
            new BeltOutakeCommand(leftBelt),
            new DiverterCommand(diverter),
            new IROutakeCommand(intakeRoller))
        .withName("Run Outake Auto");
  }

  private Command cancelActivePath() {
    return Commands.runOnce(this::cancelActivePathNow).withName("CancelActivePath");
  }

  private void cancelActivePathNow() {
    if (activePathCommand != null) {
      CommandScheduler.getInstance().cancel(activePathCommand);
      activePathCommand = null;
    }
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand(
        "AutoIntakeOn",
        Commands.defer(
            this::intakePickupHeldCommand,
            Set.of(intakePivot, intakeRoller, rightBelt, leftBelt, diverter)));

    NamedCommands.registerCommand(
        "AutoIntakeOff",
        Commands.runOnce(
            () -> {
              drive.setSpeedNormal();
              leftHood.pause();
              rightHood.pause();
            }));

    NamedCommands.registerCommand(
        "IntakeOut", Commands.defer(this::intakePickupOutCommand, Set.of(intakePivot)));

    NamedCommands.registerCommand(
        "ToggleAdaptiveHubAiming",
        Commands.defer(
            this::adaptiveHubAimingCommand,
            Set.of(rightRotater, rightHood, leftRotater, leftHood)));

    NamedCommands.registerCommand("StopDrive", Commands.defer(this::stopDrive, Set.of(drive)));

    NamedCommands.registerCommand("EjectFuel", ejectFuelCommand());

    NamedCommands.registerCommand(
        "Simply Shoot",
        Commands.runOnce(
                () -> {
                  leftShooter.setSpeed(2650);
                  rightShooter.setSpeed(2650);
                })
            .andThen(Commands.waitSeconds(1.0))
            // .andThen(
            //     Commands.parallel(
            //         new BeltIntakeCommand(rightBelt), new BeltIntakeCommand(leftBelt))));
            .andThen(beltSequence())
            .finallyDo(
                () -> {
                  rightBelt.getIO().stop();
                  leftBelt.getIO().stop();
                }));
  }

  private Command beltSequence() {
    return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> rightBelt.getIO().intake()),
                new InstantCommand(() -> leftBelt.getIO().intake())),
            new WaitCommand(0.1),
            new ParallelCommandGroup(
                new InstantCommand(() -> rightBelt.getIO().stop()),
                new InstantCommand(() -> leftBelt.getIO().stop())),
            new WaitCommand(0.5))
        .repeatedly();
  }

  private Command deferredCommand(
      java.util.function.Supplier<Command> supplier, Subsystem... requirements) {
    return Commands.defer(supplier, Set.of(requirements));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
