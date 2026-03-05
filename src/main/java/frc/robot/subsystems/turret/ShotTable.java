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
    table.put(0.00, new ShotSetpoint(0.00, 2750));
    table.put(1.00, new ShotSetpoint(0.00, 2750));
    table.put(1.13, new ShotSetpoint(0.00, 2750));
    table.put(1.44, new ShotSetpoint(0.00, 2900));
    table.put(2.03, new ShotSetpoint(0.07, 3000));
    table.put(2.33, new ShotSetpoint(0.08, 3000));
    table.put(2.65, new ShotSetpoint(0.10, 3255));
    table.put(2.98, new ShotSetpoint(0.12, 3300));
    table.put(3.33, new ShotSetpoint(0.13, 3380));
    table.put(3.71, new ShotSetpoint(0.14, 3420));
    table.put(3.96, new ShotSetpoint(0.15, 3590));
    table.put(4.48, new ShotSetpoint(0.10, 3570));
    table.put(5.13, new ShotSetpoint(0.26, 3640));
    table.put(10.00, new ShotSetpoint(0.26, 3640));
    table.put(15.00, new ShotSetpoint(0.26, 3640));
    table.put(20.00, new ShotSetpoint(0.26, 3640));
  }

  public static ShotSetpoint get(double distanceMeters) {
    return table.get(distanceMeters);
  }
}
