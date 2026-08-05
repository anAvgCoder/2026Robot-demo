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
    // Measured shot data (distance in meters, hood degrees, shooter RPM):
    //   0.76 m (30 in), 25 deg, 1500 RPM
    //   0.89 m (35 in), 25 deg, 1580 RPM
    //   1.14 m (45 in), 30 deg, 1630 RPM
    table.put(0.00, new ShotSetpoint(calculateHoodAngle(25), 1500));
    table.put(0.99, new ShotSetpoint(calculateHoodAngle(25), 1500));
    table.put(1.11, new ShotSetpoint(calculateHoodAngle(25), 1580));
    table.put(1.36, new ShotSetpoint(calculateHoodAngle(30), 1630));

    // Linear extrapolation from the three measured points above (~+320.5 RPM/m,
    // ~+14.1 deg/m of hood angle) out to 3.0 m. Untested past 1.14 m -- verify on the robot
    // before trusting these, then replace with real measurements as you collect them.
    table.put(1.78, new ShotSetpoint(calculateHoodAngle(35), 1752));
    table.put(2.02, new ShotSetpoint(calculateHoodAngle(39), 1848));
    table.put(2.33, new ShotSetpoint(calculateHoodAngle(41), 1945));
    table.put(2.63, new ShotSetpoint(calculateHoodAngle(41), 2041));
    table.put(2.93, new ShotSetpoint(calculateHoodAngle(41), 2137));
    table.put(3.23, new ShotSetpoint(calculateHoodAngle(41), 2233));

    // KEEP THESE TO REUSE LATER
    // table.put(0.00, new ShotSetpoint(0.0, 2000));
    // table.put(1.55, new ShotSetpoint(0.0, 2150));
    // table.put(1.86, new ShotSetpoint(calculateHoodAngle(22), 2150));
    // table.put(2.14, new ShotSetpoint(calculateHoodAngle(22), 2250));
    // table.put(2.45, new ShotSetpoint(calculateHoodAngle(23), 2300));
    // table.put(2.73, new ShotSetpoint(calculateHoodAngle(23), 2410));
    // table.put(3.06, new ShotSetpoint(calculateHoodAngle(24), 2470));
    // table.put(3.42, new ShotSetpoint(calculateHoodAngle(24), 2550));
    // table.put(3.74, new ShotSetpoint(calculateHoodAngle(25), 2600));
    // table.put(4.05, new ShotSetpoint(calculateHoodAngle(28), 2600));
    // table.put(4.44, new ShotSetpoint(calculateHoodAngle(28), 2650));
    // table.put(4.90, new ShotSetpoint(calculateHoodAngle(32), 2700));
    // table.put(5.34, new ShotSetpoint(calculateHoodAngle(33), 2700));
    // table.put(7.50, new ShotSetpoint(calculateHoodAngle(36), 2900));
    // table.put(10.0, new ShotSetpoint(calculateHoodAngle(41), 3600));
    // table.put(11.5, new ShotSetpoint(calculateHoodAngle(41), 4250));
    // table.put(13.5, new ShotSetpoint(calculateHoodAngle(41), 4550));
    // table.put(15.0, new ShotSetpoint(calculateHoodAngle(41), 4550));
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

  public static double calculateHoodAngle(double degrees) {
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
