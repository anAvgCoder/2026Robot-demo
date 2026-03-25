package frc.robot;

import static frc.robot.subsystems.questNav.QuestNavConstants.ROBOT_TO_QUEST;

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
import frc.robot.commands.shooter.ShooterAdaptiveAiming;
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
import frc.robot.subsystems.intakePivot.IntakePivotIOReal;
import frc.robot.subsystems.intakePivot.IntakePivotIOReplay;
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
  private static final JoystickButton resetQuestPoseBlueButton = new JoystickButton(rightJoy, 11);

  private static final JoystickButton outakeButton = new JoystickButton(leftJoy, 1);
  private static final JoystickButton leftJoy2Button = new JoystickButton(leftJoy, 2);
  private static final JoystickButton leftJoy3Button = new JoystickButton(leftJoy, 3);
  private static final JoystickButton leftJoy4Button = new JoystickButton(leftJoy, 4);

  private static final JoystickButton leftJoy14Button = new JoystickButton(leftJoy, 14);
  private static final JoystickButton leftJoy13Button = new JoystickButton(leftJoy, 13);

  private static final JoystickButton resetQuestPoseRedButton = new JoystickButton(leftJoy, 11);

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

  // SingleMotorVelocityPIDFSparkMaxTest test = new SingleMotorVelocityPIDFSparkMaxTest();

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
        // leftBelt = new Belt(new BeltIOSim(), "BeltLeft");

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
            () -> getClampedDrive(rightJoy) ? -rightJoy.getY() : 0.0,
            () -> getClampedDrive(rightJoy) ? -rightJoy.getX() : 0.0,
            () -> getClampedTurn(leftJoy) ? -leftJoy.getX() : 0.0));

    //  all this is driven off pose unless manual position is selected, then will stay manual until
    //    re-enable of auto button is selected  (start shooter in blue button bottom row will redo
    // auto run)
    //       (shooter stop will be removed in comp)
    //
    //   shooter default command
    //     shooter always runs
    //     hoods are automatic

    //   shooter should aways track robot pose for close side
    //     shooter should track to outpost and other when in mid field
    //    hoods should go down automatically when approaching trench - also disable belts
    // temporarily

    // key button board maps
    //  top right button 1        -   60 angle
    //  top middle buttone 2      -   zero angle for shooters
    //  top left button 3         -   60 angle
    //  top 2 row right button 4  -   90 angle
    //  top 2 row middle button 5 -   force hood down - maybe add belt and intake stop
    //  top 2 row left button 6   -   90 angle

    // lower panel from top
    //  row 1     turn turets set degree from current position
    //  row 2     hood up down
    //  row 3     speed up down
    //  row 4     shooter and hood enable and disable for testing only (to override default command)

    // joystick trigger right trigger will do intake and speed robot to intake pace  (intake out if
    // not out) run belts

    // joystick right button 2 (middle top below hat) is drive to angle

    // joystick button button 3 is turbo speed (100% power)
    // joystick button button 4 is turbo speed (100% power)

    //  joystick left trigger is outtake

    //  joystick left button 2 is intake in while held goes back out after

    //  joystick 3 and 4 will be sweep left and right

    //  reset blue (if no photonvision)

    //  reset red (if not photonvision)

    //  reset blue hub (no photonvision)

    // reset red hub (no photonvision)

    rightJoy1Button.whileTrue(
        Commands.parallel(
                Commands.runOnce(
                    () -> {
                      drive.setSpeedIntake();
                      leftHood.resume();
                      rightHood.resume();
                    }),
                new IPIntakeCommand(intakePivot),
                new IRIntakeCommand(intakeRoller),
                new BeltIntakeCommand(rightBelt),
                new BeltIntakeCommand(leftBelt),
                new DiverterCommand(diverter))
            .withName("Run Intake"));

    rightJoy1Button.onFalse(
        Commands.runOnce(
            () -> {
              drive.setSpeedNormal();
              leftHood.pause();
              rightHood.pause();
            }));

    // joystick right button 2 (middle top below hat) is drive to angle
    rightJoy2Button.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> getClampedDrive(rightJoy) ? -rightJoy.getY() : 0.0,
            () -> getClampedDrive(rightJoy) ? -rightJoy.getX() : 0.0));

    // joystick button button 3 is full speed
    rightJoy3Button.whileTrue(Commands.runOnce(drive::setSpeedFull).withName("set speed full"));
    rightJoy3Button.onFalse(Commands.runOnce(drive::setSpeedNormal).withName("set speed normal"));

    // joystick button button 4 is intake speed
    rightJoy4Button.whileTrue(Commands.runOnce(drive::setSpeedIntake).withName("setSpeedIntake"));
    rightJoy4Button.onFalse(Commands.runOnce(drive::setSpeedNormal).withName("setSpeedNormal"));

    //  joystick left trigger is outtake
    outakeButton.whileTrue(
        Commands.parallel(
                new BeltOutakeCommand(rightBelt),
                new BeltOutakeCommand(leftBelt),
                new DiverterCommand(diverter),
                new IROutakeCommand(intakeRoller))
            .withName("Run Outtake"));

    //  joystick left button 2 is intake in while held goes back out after
    leftJoy2Button.onTrue(new IPStorageCommand(intakePivot));
    leftJoy2Button.onFalse(new IPIntakeCommand(intakePivot));

    //  joystick 3 and 4 will be sweep left and right

    //  reset blue (if no photonvision)

    //  reset red (if not photonvision)

    //  reset blue hub (no photonvision)

    // reset red hub (no photonvision)

    // deployOutakeButton.onTrue(new IPIntakeCommand(intakePivot));

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

    // hood down belt intake stop
    panelButton5.onTrue(
        Commands.runOnce(
                () -> {
                  leftHood.pause();
                  rightHood.pause();
                  rightBelt.pause();
                  leftBelt.pause();
                  diverter.pause();
                  intakeRoller.pause();
                })
            .withName("Pause all"));

    // hood down belt intake stop resume where at
    panelButton5.onFalse(
        Commands.runOnce(
                () -> {
                  leftHood.resume();
                  rightHood.resume();
                  rightBelt.resume();
                  leftBelt.resume();
                  diverter.resume();
                  intakeRoller.resume();
                })
            .withName("Resume all"));

    // panelButton3.and(panelButton8.negate()).whileTrue(adaptiveHubNoMoveCommand());
    // panelButton6.and(panelButton8.negate()).whileTrue(adaptiveStorageNoMoveCommand());

    //  rs test    leftJoy14Button.toggleOnTrue(shooterAdaptiveAimingCommand());
    leftJoy14Button.toggleOnTrue(adaptiveHubAimingCommand());

    // leftJoy13Button.toggleOnTrue(shooterAdaptiveHubAimingCommand());

    // panelButton1.toggleOnTrue(shooterAdaptiveHubAimingCommand());
    // panelButton4.toggleOnTrue(shooterAdaptiveStorageAimingCommand());

    // panelButton5.and(panelButton8.negate()).whileTrue(adaptiveStorageAimingCommand());
    // panelButton2.and(panelButton8.negate()).whileTrue(adaptiveHubAimingCommand());

    // panelButton7.onTrue(cancelActivePath());

    // panelButton1.toggleOnTrue(
    //     Commands.runOnce(
    //         () -> {
    //           leftShooter.setSpeed(3600);
    //           rightShooter.setSpeed(3600);
    //           leftHood.setHoodPosition(0.3);
    //           rightHood.setHoodPosition(0.3);
    //         }));
    // panelButton2.toggleOnTrue(
    //     Commands.runOnce(
    //         () -> {
    //           leftShooter.setSpeed(3300);
    //           rightShooter.setSpeed(3300);
    //           leftHood.setHoodPosition(0.3);
    //           rightHood.setHoodPosition(0.3);
    //         }));
    // panelButton3.toggleOnTrue(
    //     Commands.runOnce(
    //         () -> {
    //           leftShooter.setSpeed(3000);
    //           rightShooter.setSpeed(3000);
    //           leftHood.setHoodPosition(0.3);
    //           rightHood.setHoodPosition(0.3);
    //         }));
    // panelButton1.onTrue(Commands.runOnce(() -> questSensor.toggleIgnoreFlags()));
    // panelButton2.onTrue(Commands.runOnce(() -> questSensor.clearFlags()));
    // panelButton3.onTrue(Commands.runOnce(() -> questSensor.confirmFlag()));

    // panelButton5.whileTrue(
    //    Commands.runOnce(
    //        () -> {
    //          leftShooter.setSpeed(3000);
    //          rightShooter.setSpeed(3000);
    //          leftHood.setHoodPosition(0.42);
    //          rightHood.setHoodPosition(0.42);
    //        }));

    panelButton15.onTrue(
        Commands.runOnce(
                () -> {
                  leftShooter.stop();
                  rightShooter.stop();
                  leftHood.setHoodPosition(0.0);
                  rightHood.setHoodPosition(0.0);
                  // turn automated aiming off
                })
            .withName("set hoods down"));

    panelButton14.onTrue(
        Commands.runOnce(
                () -> {
                  // this will enable auto aim and speed until manual position is set
                  leftShooter.setSpeed(2700);
                  rightShooter.setSpeed(2700);
                  leftHood.setHoodPosition(0.3);
                  rightHood.setHoodPosition(0.3);
                })
            .withName("ready shooters"));

    /*
        panelButton15.whileTrue(
            Commands.runOnce(
                () -> {
                  shotTable.setMultiFactor(shotTable.getMultiFactor() + 0.05);
                }));
        panelButton7.whileTrue(
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

    */

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

  private Command adaptiveStorageNoMoveCommand() {
    return deferredCommand(
            () ->
                new AdaptiveNoMove(
                    rightRotater,
                    rightHood,
                    leftRotater,
                    leftHood,
                    drive,
                    isBlueAlliance(),
                    shotTable),
            rightRotater,
            rightHood,
            leftRotater,
            leftHood)
        .withName("deferred AdaptiveNoMove");
  }

  private Command adaptiveStorageAimingCommand() {
    return deferredCommand(
            () ->
                new AdaptiveStorageAiming(
                    rightRotater,
                    rightHood,
                    leftRotater,
                    leftHood,
                    drive,
                    isBlueAlliance(),
                    shotTable),
            rightRotater,
            rightHood,
            leftRotater,
            leftHood)
        .withName("deferred AdaptiveStorageAiming");
  }

  private Command shooterAdaptiveAimingCommand() {
    return deferredCommand(
        () ->
            new ShooterAdaptiveAiming(
                rightRotater,
                rightShooter,
                rightHood,
                leftRotater,
                leftShooter,
                leftHood,
                drive,
                isBlueAlliance()), // , shotTable),
        rightShooter,
        leftShooter);
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
            leftShooter)
        .withName("deferred ShooterAdaptiveStorageAiming");
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
        "StartPick",
        Commands.runOnce(
                () -> {
                  intakePivot.setIntakeExtended();
                  intakeRoller.getIO().intake();
                  leftBelt.getIO().intake();
                  rightBelt.getIO().intake();
                  diverter.getIO().intake();
                })
            .withName("StartPick"));

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
