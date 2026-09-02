package frc.robot;

import frc.robot.subsystems.drive.Drive;

/** Bundle of every subsystem instance, built by {@link SubsystemFactory} for the active mode. */
record Subsystems(Drive drive) {}
