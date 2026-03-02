package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.shooter.Shooter;

public class ShooterSetSpeedCommand extends Command {
  private final Shooter shooter;
  private int runCounter;

  public ShooterSetSpeedCommand(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    runCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.getIO().setSpeed(3200);
    runCounter++;
    if (runCounter > 24) {
      runCounter = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.getIO().stopApplyingMotor();
  }

  // Returns true when the command should end.
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
