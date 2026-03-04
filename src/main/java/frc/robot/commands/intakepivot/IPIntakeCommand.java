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

  @Override
  public void initialize() {
    runCounter = 0;
  }

  @Override
  public void execute() {
    intakePivotIO.setIntakeSecondaryPosition();

    runCounter++;
    if (runCounter > 24) {
      runCounter = 0;
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakePivotIO.setStoragePosition();
  }

  @Override
  public boolean isFinished() {
    if (!DriverStation.isAutonomous()) {
      return false;
    } else {
      return runCounter == 24;
    }
  }
}
