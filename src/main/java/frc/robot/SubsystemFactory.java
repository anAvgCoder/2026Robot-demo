package frc.robot;

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
import frc.robot.subsystems.questnav.QuestNavSensor;
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

/** Builds the concrete {@link Subsystems} set for the active {@link Constants.Mode}. */
final class SubsystemFactory {
  private SubsystemFactory() {}

  static Subsystems create(Constants.Mode mode, QuestNavSensor questSensor) {
    return switch (mode) {
      case REAL -> createReal(questSensor);
      case SIM -> createSim(questSensor);
      default -> createReplay(questSensor);
    };
  }

  private static Subsystems createReal(QuestNavSensor questSensor) {
    Drive drive =
        new Drive(
            new GyroIOPigeon2(),
            questSensor,
            new ModuleIOSpark(0),
            new ModuleIOSpark(1),
            new ModuleIOSpark(2),
            new ModuleIOSpark(3));
    Vision vision = new Vision(new PhotonVisionIO(drive));
    Diverter diverter = new Diverter(new DiverterIOReal());
    Belt leftBelt = new Belt(new BeltIOReal(BeltConstants.CanIdLeft), "BeltLeft");
    Belt rightBelt = new Belt(new BeltIOReal(BeltConstants.CanIdRight), "BeltRight");
    IntakePivot intakePivot =
        new IntakePivot(
            new IntakePivotIOReal(
                IntakePivotConstants.kCanId, IntakePivotConstants.kSwitchDIOChannel));
    IntakeRoller intakeRoller = new IntakeRoller(new IntakeRollerIOReal());

    Shooter leftShooter = new Shooter(new ShooterIOReal(45), "ShooterLeft");
    Rotater leftRotater =
        new Rotater(
            new RotaterIOReal(RotaterConstants.kCanIdLeft, RotaterConstants.kCanIdLeftCoder),
            "RotaterLeft");
    Hood leftHood = new Hood(new HoodIOReal(HoodConstants.kLeftCanId), "HoodLeft");

    Shooter rightShooter = new Shooter(new ShooterIOReal(44), "ShooterRight");
    Rotater rightRotater =
        new Rotater(
            new RotaterIOReal(RotaterConstants.kCanIdRight, RotaterConstants.kCanIdRightCoder),
            "RotaterRight");
    Hood rightHood = new Hood(new HoodIOReal(HoodConstants.kRightCanId), "HoodRight");

    return new Subsystems(
        drive,
        vision,
        diverter,
        leftBelt,
        rightBelt,
        intakePivot,
        intakeRoller,
        leftRotater,
        leftShooter,
        leftHood,
        rightRotater,
        rightShooter,
        rightHood);
  }

  private static Subsystems createSim(QuestNavSensor questSensor) {
    Drive drive =
        new Drive(
            new GyroIO() {},
            questSensor,
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
    Vision vision = new Vision(new PhotonVisionSimIO());
    Diverter diverter = new Diverter(new DiverterIOSim());
    Belt leftBelt = new Belt(new BeltIOSim(), "BeltLeft");
    Belt rightBelt = new Belt(new BeltIOSim(), "BeltRight");
    IntakePivot intakePivot = new IntakePivot(new IntakePivotIOSim());
    IntakeRoller intakeRoller = new IntakeRoller(new IntakeRollerIOSim());

    Shooter leftShooter = new Shooter(new ShooterIOSim());
    Rotater leftRotater = new Rotater(new RotaterIOSim(), "LeftRotater");
    Hood leftHood = new Hood(new HoodIOSim(), "LeftHood");

    Shooter rightShooter = new Shooter(new ShooterIOSim());
    Rotater rightRotater = new Rotater(new RotaterIOSim(), "RightRotater");
    Hood rightHood = new Hood(new HoodIOSim(), "RightHood");

    return new Subsystems(
        drive,
        vision,
        diverter,
        leftBelt,
        rightBelt,
        intakePivot,
        intakeRoller,
        leftRotater,
        leftShooter,
        leftHood,
        rightRotater,
        rightShooter,
        rightHood);
  }

  private static Subsystems createReplay(QuestNavSensor questSensor) {
    Drive drive =
        new Drive(
            new GyroIO() {},
            questSensor,
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    Vision vision = new Vision(new PhotonVisionSimIO());
    Diverter diverter = new Diverter(new DiverterIOSim());
    Belt leftBelt = new Belt(new BeltIOSim() {}, "BeltLeft");
    Belt rightBelt = new Belt(new BeltIOSim() {}, "BeltRIght");
    IntakePivot intakePivot = new IntakePivot(new IntakePivotIOReplay());
    IntakeRoller intakeRoller = new IntakeRoller(new IntakeRollerIO() {});

    Shooter leftShooter = new Shooter(new ShooterIOSim());
    Rotater leftRotater = new Rotater(new RotaterIO() {}, "LeftRotater");
    Hood leftHood = new Hood(new HoodIO() {}, "LeftHood");

    Shooter rightShooter = new Shooter(new ShooterIOSim());
    Rotater rightRotater = new Rotater(new RotaterIO() {}, "RightRotater");
    Hood rightHood = new Hood(new HoodIO() {}, "RightHood");

    return new Subsystems(
        drive,
        vision,
        diverter,
        leftBelt,
        rightBelt,
        intakePivot,
        intakeRoller,
        leftRotater,
        leftShooter,
        leftHood,
        rightRotater,
        rightShooter,
        rightHood);
  }
}
