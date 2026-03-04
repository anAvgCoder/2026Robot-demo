package frc.robot.subsystems.turret;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShotTimeTable {
  private static final InterpolatingTreeMap<Double, Double> table =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), (a, b, t) -> a + (b - a) * t);

  static {
    // key = distanceMeters, value = flightTimeSeconds
    table.put(0.00, 0.95);
    table.put(1.00, 0.95);
    table.put(1.13, 0.95);
    table.put(1.44, 1.05);
    table.put(2.03, 1.05);
    table.put(2.33, 1.35);
    table.put(2.65, 1.56);
    table.put(2.98, 1.64);
    table.put(3.33, 1.70);
    table.put(3.71, 1.67);
    table.put(3.96, 1.62);
    table.put(4.48, 1.70);
    table.put(5.13, 1.70);
  }

  public static double getFlightTimeSeconds(double distanceMeters) {
    return table.get(distanceMeters);
  }
}
