package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShotTable {
  public record ShotSetpoint(double hoodPos, double shooterSpeed) {}

  private static final Interpolator<ShotSetpoint> shotInterpolator =
      (a, b, t) ->
          new ShotSetpoint(
              MathUtil.interpolate(a.hoodPos(), b.hoodPos(), t),
              MathUtil.interpolate(a.shooterSpeed(), b.shooterSpeed(), t));

  private static final InterpolatingTreeMap<Double, ShotSetpoint> table =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), shotInterpolator);

  static {
    // key = distanceMeters
    table.put(0.00, new ShotSetpoint(0.00, 2300));
    table.put(1.00, new ShotSetpoint(0.00, 2300));
    table.put(1.13, new ShotSetpoint(0.00, 2300));
    table.put(1.44, new ShotSetpoint(0.00, 2450));
    table.put(2.03, new ShotSetpoint(0.07, 2550));
    table.put(2.33, new ShotSetpoint(0.08, 2450));
    table.put(2.65, new ShotSetpoint(0.10, 2800));
    table.put(2.98, new ShotSetpoint(0.12, 2850));
    table.put(3.33, new ShotSetpoint(0.13, 2930));
    table.put(3.71, new ShotSetpoint(0.14, 2970));
    table.put(3.96, new ShotSetpoint(0.15, 3140));
    table.put(4.48, new ShotSetpoint(0.10, 3120));
    table.put(5.13, new ShotSetpoint(0.26, 3490));
    table.put(10.00, new ShotSetpoint(0.26, 3750));
    table.put(15.00, new ShotSetpoint(0.26, 3890));
    table.put(20.00, new ShotSetpoint(0.26, 3990));
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
    return table.get(distanceMeters);
  }
}
