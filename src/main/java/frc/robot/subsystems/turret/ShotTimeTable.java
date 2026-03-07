package frc.robot.subsystems.turret;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShotTimeTable {
  private static final InterpolatingTreeMap<Double, Double> table =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), (a, b, t) -> a + (b - a) * t);

  static {
    // key = distanceMeters, value = flightTimeSeconds
    table.put(0.00, 1.23);
    table.put(1.55, 1.23);
    table.put(1.86, 1.24);
    table.put(2.14, 1.31);
    table.put(2.45, 1.28);
    table.put(2.73, 1.34);
    table.put(3.06, 1.38);
    table.put(3.42, 1.40);
    table.put(3.74, 1.21);
    table.put(4.05, 1.36);
    table.put(4.44, 1.34);
    table.put(4.90, 1.42);
    table.put(5.34, 1.30);
    table.put(7.50, 1.5);
    table.put(10.00, 1.60);
    table.put(15.00, 1.70);
  }

  public static double getFlightTimeSeconds(double distanceMeters) {
    return table.get(distanceMeters);
  }
}
