package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.subsystems.turret.hood.HoodConstants;

public class ShotTable {
  private static double shooterMultiFactor = 1.0;

  public record ShotSetpoint(double hoodPos, double shooterSpeed) {

  public ShotSetpoint trimSetpoint () {
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
    // key = distanceMeters
    table.put(0.00, new ShotSetpoint(0.0, 2235));
    table.put(1.55, new ShotSetpoint(0.0, 2235));
    table.put(1.86, new ShotSetpoint(calculateHoodAngle(17), 2260));
    table.put(2.14, new ShotSetpoint(calculateHoodAngle(18), 2310 ));
    table.put(2.45, new ShotSetpoint(calculateHoodAngle(19.5), 2360));
    table.put(2.73, new ShotSetpoint(calculateHoodAngle(20.5), 2410));
    table.put(3.06, new ShotSetpoint(calculateHoodAngle(22), 2510 ));
    table.put(3.42, new ShotSetpoint(calculateHoodAngle(23), 2600 ));
    table.put(3.74, new ShotSetpoint(calculateHoodAngle(24), 2660));
    table.put(4.05, new ShotSetpoint(calculateHoodAngle(25), 2725 ));
    table.put(4.44, new ShotSetpoint(calculateHoodAngle(25.5), 2725));
    table.put(4.90, new ShotSetpoint(calculateHoodAngle(26), 2740));
    table.put(5.34, new ShotSetpoint(calculateHoodAngle(27), 2770));
    table.put(7.50, new ShotSetpoint(calculateHoodAngle(27), 4000));
    table.put(10.0, new ShotSetpoint(calculateHoodAngle(27), 4000));
    table.put(12.5, new ShotSetpoint(calculateHoodAngle(27), 4000));
    table.put(15.0, new ShotSetpoint(calculateHoodAngle(27), 4000));
    // table.put(0.00, new ShotSetpoint(0.00, 2700));
    // table.put(1.00, new ShotSetpoint(0.00, 2700));
    // table.put(1.13, new ShotSetpoint(0.00, 2700));
    // table.put(1.44, new ShotSetpoint(0.00, 2850));
    // table.put(2.03, new ShotSetpoint(0.07, 2950));
    // table.put(2.33, new ShotSetpoint(0.08, 2950));
    // table.put(2.65, new ShotSetpoint(0.10, 3200));
    // table.put(2.98, new ShotSetpoint(0.12, 3250));
    // table.put(3.33, new ShotSetpoint(0.13, 3330));
    // table.put(3.71, new ShotSetpoint(0.14, 3370));
    // table.put(3.96, new ShotSetpoint(0.15, 3540));
    // table.put(4.48, new ShotSetpoint(0.10, 3520));
    // table.put(5.13, new ShotSetpoint(0.26, 3590));
    // table.put(10.00, new ShotSetpoint(0.26, 3590));
    // table.put(15.00, new ShotSetpoint(0.26, 3590));
    // table.put(20.00, new ShotSetpoint(0.26, 3590));
  }

  public static ShotSetpoint get(double distanceMeters) {
    return table.get(distanceMeters).trimSetpoint()  ;
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
        (normalized * HoodConstants.kMaxAngleRad + max - min), 0.0, HoodConstants.kMaxAngleRad);
  }
}
