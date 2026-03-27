package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.subsystems.turret.hood.HoodConstants;

public class ShotTable {
  private static double shooterMultiFactor = 1.15;

  public record ShotSetpoint(double hoodPos, double shooterSpeed) {
    public ShotSetpoint trimSetpoint() {
      return new ShotSetpoint(hoodPos, shooterSpeed * shooterMultiFactor);
    }
  }

  private static final Interpolator<ShotSetpoint> shotInterpolator =
      (a, b, t) ->
          new ShotSetpoint(
              MathUtil.interpolate(a.hoodPos(), b.hoodPos(), t),
              MathUtil.interpolate(a.shooterSpeed(), b.shooterSpeed(), t));

  private static final InterpolatingTreeMap<Double, ShotSetpoint> table =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), shotInterpolator);

  static {
    table.put(0.00, new ShotSetpoint(0.0, 2000));
    table.put(1.55, new ShotSetpoint(0.0, 2150));
    table.put(1.86, new ShotSetpoint(calculateHoodAngle(22), 2150));
    table.put(2.14, new ShotSetpoint(calculateHoodAngle(22), 2250));
    table.put(2.45, new ShotSetpoint(calculateHoodAngle(23), 2300));
    table.put(2.73, new ShotSetpoint(calculateHoodAngle(23), 2410));
    table.put(3.06, new ShotSetpoint(calculateHoodAngle(24), 2470));
    table.put(3.42, new ShotSetpoint(calculateHoodAngle(24), 2550));
    table.put(3.74, new ShotSetpoint(calculateHoodAngle(25), 2600));
    table.put(4.05, new ShotSetpoint(calculateHoodAngle(28), 2600));
    table.put(4.44, new ShotSetpoint(calculateHoodAngle(28), 2650));
    table.put(4.90, new ShotSetpoint(calculateHoodAngle(32), 2700));
    table.put(5.34, new ShotSetpoint(calculateHoodAngle(33), 2700));
    table.put(7.50, new ShotSetpoint(calculateHoodAngle(36), 2900));
    table.put(10.0, new ShotSetpoint(calculateHoodAngle(41), 3600));
    table.put(12.5, new ShotSetpoint(calculateHoodAngle(41), 3600));
    table.put(15.0, new ShotSetpoint(calculateHoodAngle(41), 3600));
  }

  public static ShotSetpoint get(double distanceMeters) {
    return table.get(distanceMeters).trimSetpoint();
  }

  public void setMultiFactor(double num) {
    shooterMultiFactor = num;
  }

  public double getMultiFactor() {
    return shooterMultiFactor;
  }

  private static double calculateHoodAngle(double degrees) {
    if (degrees < 17.5) {
      return 0.0;
    }
    if (degrees > 40.0) {
      return HoodConstants.kMaxAngleRad;
    }

    double min = 17.6;
    double max = 39.8;
    double normalized = (degrees - min) / (max - min);

    return MathUtil.clamp(
        (normalized * HoodConstants.kMaxAngleRad), 0.0, HoodConstants.kMaxAngleRad);
  }
}
