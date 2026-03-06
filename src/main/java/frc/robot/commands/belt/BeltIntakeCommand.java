package frc.robot.commands.belt;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.belt.Belt;
import frc.robot.subsystems.belt.BeltIO;

public class BeltIntakeCommand extends Command {
  private final BeltIO beltIO;
  private int runCounter;

  public BeltIntakeCommand(Belt belt) {
    beltIO = belt.getIO();
    addRequirements(belt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    beltIO.intake();
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
    beltIO.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
