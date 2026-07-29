package frc.robot;

import frc.robot.subsystems.belt.Belt;
import frc.robot.subsystems.diverter.Diverter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakeroller.IntakeRoller;
import frc.robot.subsystems.turret.hood.Hood;
import frc.robot.subsystems.turret.rotater.Rotater;
import frc.robot.subsystems.turret.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

/** Bundle of every subsystem instance, built by {@link SubsystemFactory} for the active mode. */
record Subsystems(
    Drive drive,
    Vision vision,
    Diverter diverter,
    Belt leftBelt,
    Belt rightBelt,
    IntakePivot intakePivot,
    IntakeRoller intakeRoller,
    Rotater leftRotater,
    Shooter leftShooter,
    Hood leftHood,
    Rotater rightRotater,
    Shooter rightShooter,
    Hood rightHood) {}
