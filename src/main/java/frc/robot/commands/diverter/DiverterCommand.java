package frc.robot.commands.diverter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.diverter.Diverter;
import frc.robot.subsystems.diverter.DiverterIO;

public class DiverterCommand extends Command {
  private final DiverterIO diverterIO;
  private int runCounter;

  public DiverterCommand(Diverter diverter) {
    diverterIO = diverter.getIO();
    addRequirements(diverter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    runCounter = 0;
    diverterIO.intake();
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
    diverterIO.stop();
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
