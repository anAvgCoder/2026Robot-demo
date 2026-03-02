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
    table.put(1.0, new ShotSetpoint(0, 0.0));
    table.put(5.0, new ShotSetpoint(0.7, 0.0));
    table.put(10.0, new ShotSetpoint(1.4, 0.0));
  }

  public static ShotSetpoint get(double distanceMeters) {
    return table.get(distanceMeters);
  }
}
