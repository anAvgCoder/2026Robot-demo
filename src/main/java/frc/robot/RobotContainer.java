package frc.robot;

import static frc.robot.subsystems.questnav.QuestNavConstants.ROBOT_TO_QUEST;

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
import frc.robot.commands.DriveCommands;
import frc.robot.commands.belt.BeltIntakeCommand;
import frc.robot.commands.belt.BeltOutakeCommand;
import frc.robot.commands.diverter.DiverterCommand;
import frc.robot.commands.intakepivot.IPIntakeCommand;
import frc.robot.commands.intakeroller.IRIntakeCommand;
import frc.robot.commands.intakeroller.IROutakeCommand;
import frc.robot.commands.turret.AdaptiveHubAiming;
import frc.robot.oi.LeftStick;
import frc.robot.oi.OperatorPanel;
import frc.robot.oi.RightStick;
import frc.robot.subsystems.belt.Belt;
import frc.robot.subsystems.diverter.Diverter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakeroller.IntakeRoller;
import frc.robot.subsystems.questnav.QuestNavConstants;
import frc.robot.subsystems.questnav.QuestNavSensor;
import frc.robot.subsystems.turret.ShotTable;
import frc.robot.subsystems.turret.hood.Hood;
import frc.robot.subsystems.turret.rotater.Rotater;
import frc.robot.subsystems.turret.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FieldConstants;
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

  // Operator interface
  private final LeftStick leftStick = new LeftStick();
  private final RightStick rightStick = new RightStick();
  private final OperatorPanel operatorPanel = new OperatorPanel();

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

    Subsystems subsystems = SubsystemFactory.create(Constants.currentMode, questSensor);
    drive = subsystems.drive();
    vision = subsystems.vision();
    diverter = subsystems.diverter();
    leftBelt = subsystems.leftBelt();
    rightBelt = subsystems.rightBelt();
    intakePivot = subsystems.intakePivot();
    intakeRoller = subsystems.intakeRoller();
    leftRotater = subsystems.leftRotater();
    leftShooter = subsystems.leftShooter();
    leftHood = subsystems.leftHood();
    rightRotater = subsystems.rightRotater();
    rightShooter = subsystems.rightShooter();
    rightHood = subsystems.rightHood();

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
            () -> getClampedDrive(rightStick.joystick) ? -rightStick.joystick.getY() : 0.0,
            () -> getClampedDrive(rightStick.joystick) ? -rightStick.joystick.getX() : 0.0,
            () -> getClampedTurn(leftStick.joystick) ? -leftStick.joystick.getX() : 0.0));

    configureDriveBindings();
    configureIntakeBindings();
    configureTurretPresetBindings();
    configureTurretAimBindings();
    configurePauseResumeBindings();
    configurePoseSourceBindings();
    configureManualNudgeBindings();
    configurePoseResetBindings();
  }

  private void configureDriveBindings() {
    // Right Joystick Button 2: lock drive heading while held
    rightStick.lockHeadingButton.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> getClampedDrive(rightStick.joystick) ? -rightStick.joystick.getY() : 0.0,
            () -> getClampedDrive(rightStick.joystick) ? -rightStick.joystick.getX() : 0.0));

    // Right Joystick Button 3: full drive speed while held, back to normal on release
    rightStick.fullSpeedButton.whileTrue(
        Commands.runOnce(drive::setSpeedFull).withName("set speed full"));
    rightStick.fullSpeedButton.onFalse(
        Commands.runOnce(drive::setSpeedNormal).withName("set speed normal"));

    // Right Joystick Button 4: intake drive speed while held, back to normal on release
    rightStick.intakeSpeedButton.whileTrue(
        Commands.runOnce(drive::setSpeedIntake).withName("setSpeedIntake"));
    rightStick.intakeSpeedButton.onFalse(
        Commands.runOnce(drive::setSpeedNormal).withName("setSpeedNormal"));
  }

  private void configureIntakeBindings() {
    // Right Joystick Button 1: run full intake sequence while held, stop it on release
    rightStick.intakeButton.whileTrue(intakePickupHeldCommand());
    rightStick.intakeButton.onFalse(intakePickupReleaseCommand());

    // Left Joystick Button 1: run belts/diverter/roller in reverse to eject fuel while held
    leftStick.outtakeButton.whileTrue(
        Commands.parallel(
                new BeltOutakeCommand(rightBelt),
                new BeltOutakeCommand(leftBelt),
                new DiverterCommand(diverter),
                new IROutakeCommand(intakeRoller))
            .withName("Run Outtake"));
  }

  private void configureTurretPresetBindings() {
    // Panel Button 2: point turret straight forward (0 deg)
    operatorPanel.rotateTo0Button.onTrue(
        Commands.runOnce(
                () -> {
                  leftRotater.setTurnPosition(0);
                  rightRotater.setTurnPosition(0);
                })
            .withName("RotateTo:0"));

    // Panel Button 1: point turret to -60 deg
    operatorPanel.rotateToNeg60Button.onTrue(
        Commands.runOnce(
                () -> {
                  leftRotater.setTurnPosition(-60);
                  rightRotater.setTurnPosition(-60);
                })
            .withName("RotateTo:-60"));

    // Panel Button 4: point turret to -90 deg
    operatorPanel.rotateToNeg90Button.onTrue(
        Commands.runOnce(
                () -> {
                  leftRotater.setTurnPosition(-90);
                  rightRotater.setTurnPosition(-90);
                })
            .withName("RotateTo:-90"));

    // Panel Button 3: point turret to 60 deg
    operatorPanel.rotateTo60Button.onTrue(
        Commands.runOnce(
                () -> {
                  leftRotater.setTurnPosition(60);
                  rightRotater.setTurnPosition(60);
                })
            .withName("RotateTo:60"));

    // Panel Button 6: point turret to 90 deg
    operatorPanel.rotateTo90Button.onTrue(
        Commands.runOnce(
                () -> {
                  leftRotater.setTurnPosition(90);
                  rightRotater.setTurnPosition(90);
                })
            .withName("RotateTo:90"));
  }

  private void configureTurretAimBindings() {
    // Left Joystick Button 5: toggle automatic hub-aiming turret control
    leftStick.toggleAutoAimButton.toggleOnTrue(adaptiveHubAimingCommand());
  }

  private void configurePauseResumeBindings() {
    // Panel Button 5: pause hoods/belts/diverter/roller while held
    operatorPanel.pauseAllButton.onTrue(
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

    // Panel Button 5: resume hoods/belts/diverter/roller on release
    operatorPanel.pauseAllButton.onFalse(
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
  }

  private void configurePoseSourceBindings() {
    // Panel Button 7: switch drive pose estimation to camera vision
    operatorPanel.switchToCameraButton.onTrue(
        Commands.runOnce(
                () -> {
                  drive.switchToCamera();
                })
            .ignoringDisable(true));

    // Panel Button 8: switch drive pose estimation to QuestNav
    operatorPanel.switchToQuestButton.onTrue(
        Commands.runOnce(
                () -> {
                  drive.switchToQuest();
                })
            .ignoringDisable(true));
  }

  private void configureManualNudgeBindings() {
    // Panel Button 9: nudge both hoods down by one manual increment
    operatorPanel.hoodDownButton.onTrue(
        Commands.runOnce(
            () -> {
              leftHood.addAngleDeg(-Constants.manualHoodIncDegrees);
              rightHood.addAngleDeg(-Constants.manualHoodIncDegrees);
            }));

    // Panel Button 11: nudge both hoods up by one manual increment
    operatorPanel.hoodUpButton.onTrue(
        Commands.runOnce(
            () -> {
              leftHood.addAngleDeg(Constants.manualHoodIncDegrees);
              rightHood.addAngleDeg(Constants.manualHoodIncDegrees);
            }));

    // Panel Button 12: nudge turret rotation positive by one manual increment
    operatorPanel.turretNudgePositiveButton.onTrue(
        Commands.runOnce(
            () -> {
              leftRotater.addTurnPosition(Constants.manualRotationIncDegrees);
              rightRotater.addTurnPosition(Constants.manualRotationIncDegrees);
            }));

    // Panel Button 13: nudge turret rotation negative by one manual increment
    operatorPanel.turretNudgeNegativeButton.onTrue(
        Commands.runOnce(
            () -> {
              leftRotater.addTurnPosition(-Constants.manualHoodIncDegrees);
              rightRotater.addTurnPosition(-Constants.manualHoodIncDegrees);
            }));

    // Panel Button 14: nudge both shooter velocities down by one manual increment
    operatorPanel.shooterNudgeDownButton.onTrue(
        Commands.runOnce(
            () -> {
              leftShooter.addVelocityRPM(-Constants.manualShooterIncRPM);
              rightShooter.addVelocityRPM(-Constants.manualShooterIncRPM);
            }));

    // Panel Button 15: nudge both shooter velocities up by one manual increment
    operatorPanel.shooterNudgeUpButton.onTrue(
        Commands.runOnce(
            () -> {
              leftShooter.addVelocityRPM(Constants.manualShooterIncRPM);
              rightShooter.addVelocityRPM(Constants.manualShooterIncRPM);
            }));
  }

  private void configurePoseResetBindings() {
    // Left Joystick Button 11: reset drive pose to the red alliance QuestNav reference point
    leftStick.resetQuestPoseRedButton.onTrue(
        Commands.runOnce(
                () ->
                    drive.setPose(
                        QuestNavConstants.ROBOT_TO_QUEST_RED.transformBy(ROBOT_TO_QUEST.inverse())))
            .ignoringDisable(true)
            .withName("ResetPoseRed"));

    // Right Joystick Button 11: reset drive pose to the blue alliance QuestNav reference point
    rightStick.resetQuestPoseBlueButton.onTrue(
        Commands.runOnce(
                () ->
                    drive.setPose(
                        QuestNavConstants.ROBOT_TO_QUEST_BLUE.transformBy(
                            ROBOT_TO_QUEST.inverse())))
            .ignoringDisable(true)
            .withName("ResetPoseBlue"));

    // Right Joystick Button 5: reset drive pose to the blue hub QuestNav reference point
    rightStick.resetQuestPoseBlueHubButton.onTrue(
        Commands.runOnce(() -> drive.setPose(FieldConstants.TRASH_CAN_POSE3D))
            .ignoringDisable(true)
            .withName("ResetPoseBlue"));

    // Right Joystick Button 7: reset drive pose to the blue hub QuestNav reference point
    rightStick.resetPoseBlueHubButton.onTrue(
        Commands.runOnce(
                () ->
                    drive.setPose(
                        QuestNavConstants.ROBOT_TO_QUEST_BLUE_HUB.transformBy(
                            ROBOT_TO_QUEST.inverse())))
            .ignoringDisable(true)
            .withName("ResetPoseBlueHub"));
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
    // return Commands.parallel(new IPIntakeCommand(intakePivot).withName("Run IntakePivot Out"));
    return Commands.runOnce(() -> {});
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
