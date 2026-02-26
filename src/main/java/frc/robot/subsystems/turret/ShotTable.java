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

  public ShotTable() {
    // key = distanceMeters
    table.put(1.0, new ShotSetpoint(0, 0.1));
    table.put(2.0, new ShotSetpoint(1, 0.2));
    table.put(3.5, new ShotSetpoint(2, 0.3));
  }

  public static ShotSetpoint get(double distanceMeters) {
    return table.get(distanceMeters);
  }
}
