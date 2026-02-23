package frc.robot.commands.intakeroller;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeroller.IntakeRoller;
import frc.robot.subsystems.intakeroller.IntakeRollerIO;

public class IRIntakeCommand extends Command {
  private final IntakeRollerIO intakeRollerIO;
  private int runCounter;

  public IRIntakeCommand(IntakeRoller intakeRoller) {
    intakeRollerIO = intakeRoller.getIO();
    addRequirements(intakeRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeRollerIO.intake();
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
    intakeRollerIO.stop();
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
