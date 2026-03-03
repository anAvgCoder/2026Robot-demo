package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.hood.Hood;
import frc.robot.subsystems.turret.shooter.Shooter;
import frc.robot.util.LoggedTunableNumber;

public class TestHoodShooterCommand extends Command {
  private final Shooter shooter;
  private final Hood hood;
  private int runCounter;

  private final LoggedTunableNumber shooterSpeed =
      new LoggedTunableNumber("TestHoodShooter/ShooterSpeed", 3200);
  private final LoggedTunableNumber hoodPosition =
      new LoggedTunableNumber("TestHoodShooter/HoodPosition", 0.0);

  public TestHoodShooterCommand(Shooter shooter, Hood hood) {
    this.shooter = shooter;
    this.hood = hood;
    addRequirements(shooter, hood);
  }

  @Override
  public void initialize() {
    runCounter = 0;
  }

  @Override
  public void execute() {
    double getShooterSpeed = MathUtil.clamp(shooterSpeed.get(), 0, 5000);
    double getHoodPosition = MathUtil.clamp(hoodPosition.get(), -1.47, 0);
    shooter.getIO().setSpeed(getShooterSpeed);
    hood.getIO().setHoodPosition(getHoodPosition);
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
