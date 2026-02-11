package frc.robot.subsystems.TurretRight.TRShooter;

import frc.robot.util.LoggedTunableNumber;

public class TRShooterConstants {
  private final LoggedTunableNumber kP = new LoggedTunableNumber("SingleMotorTest/kP", 0.00006);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("SingleMotorTest/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("SingleMotorTest/kD", 0.0);

  private final LoggedTunableNumber kS = new LoggedTunableNumber("SingleMotorTest/kS", 0.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("SingleMotorTest/kV", 0.00183);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("SingleMotorTest/kA", 0.0);
}
