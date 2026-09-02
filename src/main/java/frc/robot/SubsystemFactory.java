package frc.robot;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;

/** Builds the concrete {@link Subsystems} set for the active {@link Constants.Mode}. */
final class SubsystemFactory {
  private SubsystemFactory() {}

  static Subsystems create(Constants.Mode mode) {
    return switch (mode) {
      case REAL -> createReal();
      case SIM -> createSim();
      default -> createReplay();
    };
  }

  private static Subsystems createReal() {
    Drive drive =
        new Drive(
            new GyroIOPigeon2(),
            new ModuleIOSpark(0),
            new ModuleIOSpark(1),
            new ModuleIOSpark(2),
            new ModuleIOSpark(3));

    return new Subsystems(drive);
  }

  private static Subsystems createSim() {
    Drive drive =
        new Drive(
            new GyroIO() {},
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());

    return new Subsystems(drive);
  }

  private static Subsystems createReplay() {
    Drive drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});

    return new Subsystems(drive);
  }
}
