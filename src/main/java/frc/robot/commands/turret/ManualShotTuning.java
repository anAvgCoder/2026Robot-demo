package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.ShotTable;
import frc.robot.subsystems.turret.hood.Hood;
import frc.robot.subsystems.turret.shooter.Shooter;
import java.util.function.DoubleSupplier;

/**
 * Drives both shooters/hoods to a hand-entered RPM and angle, for collecting data points while
 * building a new {@link frc.robot.subsystems.turret.ShotTable}.
 */
public class ManualShotTuning extends Command {
  private final Shooter shooterRight;
  private final Shooter shooterLeft;
  private final Hood hoodRight;
  private final Hood hoodLeft;
  private final DoubleSupplier rpmSupplier;
  private final DoubleSupplier hoodDegSupplier;

  public ManualShotTuning(
      Shooter shooterRight,
      Shooter shooterLeft,
      Hood hoodRight,
      Hood hoodLeft,
      DoubleSupplier rpmSupplier,
      DoubleSupplier hoodDegSupplier) {
    this.shooterRight = shooterRight;
    this.shooterLeft = shooterLeft;
    this.hoodRight = hoodRight;
    this.hoodLeft = hoodLeft;
    this.rpmSupplier = rpmSupplier;
    this.hoodDegSupplier = hoodDegSupplier;
    addRequirements(shooterRight, shooterLeft, hoodRight, hoodLeft);
  }

  @Override
  public void execute() {
    double rpm = rpmSupplier.getAsDouble();
    double hoodPositionRad = ShotTable.calculateHoodAngle(hoodDegSupplier.getAsDouble());
    shooterRight.setVelocityRPM(rpm);
    shooterLeft.setVelocityRPM(rpm);
    hoodRight.setHoodPosition(hoodPositionRad);
    hoodLeft.setHoodPosition(hoodPositionRad);
  }

  @Override
  public void end(boolean interrupted) {
    shooterRight.stop();
    shooterLeft.stop();
    hoodRight.setStoragePosition();
    hoodLeft.setStoragePosition();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
