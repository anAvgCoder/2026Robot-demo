package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.belt.Belt;
import frc.robot.subsystems.turret.hood.Hood;
import frc.robot.subsystems.turret.shooter.Shooter;
import frc.robot.util.LoggedTunableNumber;

public class TestHoodShooterCommand extends Command {
  private final Shooter shooter;
  private final Hood hood;
  private final Belt belt;
  private int runCounter;

  private final LoggedTunableNumber shooterSpeed =
      new LoggedTunableNumber("TestHoodShooter/ShooterSpeed", 1600);
  private final LoggedTunableNumber hoodPosition =
      new LoggedTunableNumber("TestHoodShooter/HoodPosition", 0.0);
  private final LoggedTunableNumber beltOn = new LoggedTunableNumber("TestHoodShooter/BeltOn", 0.0);

  public TestHoodShooterCommand(Shooter shooter, Hood hood, Belt belt) {
    this.shooter = shooter;
    this.hood = hood;
    this.belt = belt;
    addRequirements(shooter, hood, belt);
  }

  @Override
  public void initialize() {
    runCounter = 0;
  }

  @Override
  public void execute() {
    double getShooterSpeed = MathUtil.clamp(shooterSpeed.get(), 0, 5000);
    shooter.getIO().setSpeed(getShooterSpeed);
    hood.getIO().setHoodPosition(hoodPosition.get());

    if (beltOn.get() > 0.5) {
      belt.getIO().intake();
      ;
    } else {
      belt.getIO().stop();
    }
    runCounter++;
    if (runCounter > 24) {
      runCounter = 0;
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.getIO().stopApplyingMotor();
  }

  @Override
  public boolean isFinished() {
    if (!DriverStation.isAutonomous()) {
      return false;
    } else {
      if (runCounter == 24) {
        return true;
      } else {
        return false;
      }
    }
  }
}
