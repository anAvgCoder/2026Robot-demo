package frc.robot.commands.intakeroller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeroller.IntakeRoller;

public class IRIntakeCommand extends Command {
  private final IntakeRoller intakeRoller;
  private int runCounter;

  public IRIntakeCommand(IntakeRoller intakeRoller) {
    this.intakeRoller = intakeRoller;
    addRequirements(intakeRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    runCounter = 0;
    intakeRoller.intake();
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
    intakeRoller.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
