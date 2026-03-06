package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakepivot.IntakePivotIO;

public class IPStorageCommand extends Command {
  private final IntakePivotIO intakePivotIO;
  private int runCounter;

  public IPStorageCommand(IntakePivot intakePivot) {
    intakePivotIO = intakePivot.getIO();
    addRequirements(intakePivot);
  }

  @Override
  public void initialize() {
    runCounter = 0;
  }

  @Override
  public void execute() {
    intakePivotIO.setStoragePosition();

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
    return false;
  }
}
