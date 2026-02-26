package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakepivot.IntakePivotIO;

public class IPIntakeCommand extends Command {
  private final IntakePivotIO intakePivotIO;
  private int runCounter;

  public IPIntakeCommand(IntakePivot intakePivot) {
    intakePivotIO = intakePivot.getIO();
    addRequirements(intakePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    runCounter = 0;
    intakePivotIO.setIntakePosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    runCounter++;
    if (runCounter > 24) {
      runCounter = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakePivotIO.setStoragePosition();
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
