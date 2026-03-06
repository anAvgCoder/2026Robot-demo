package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.shooter.Shooter;

public class ShooterSetSpeedCommand extends Command {
  private final Shooter shooter;
  private final double targetRPM;

  public ShooterSetSpeedCommand(Shooter shooter) {
    this(shooter, 3200.0);
  }

  public ShooterSetSpeedCommand(Shooter shooter, double targetRPM) {
    this.shooter = shooter;
    this.targetRPM = targetRPM;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.setVelocityRPM(targetRPM);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
