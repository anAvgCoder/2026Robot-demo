package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakePivot;

public class IPIntakeCommand extends Command {
  private final IntakePivot intakePivot;
  private int runCounter;

  public IPIntakeCommand(IntakePivot intakePivot) {
    this.intakePivot = intakePivot;
    addRequirements(intakePivot);
  }

  @Override
  public void initialize() {
    runCounter = 0;
  }

  @Override
  public void execute() {
    intakePivot.setIntakeSecondaryPosition();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
