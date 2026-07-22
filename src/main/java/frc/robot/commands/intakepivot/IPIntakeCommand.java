package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakepivot.IntakePivot;

public class IPIntakeCommand extends Command {
  private final IntakePivot intakePivot;
  private int runCounter;

  public IPIntakeCommand(IntakePivot intakePivot) {
    this.intakePivot = intakePivot;
    addRequirements(intakePivot);
  }

  @Override
  public void initialize() {
    // intakePivot.setIntakeExtended();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
