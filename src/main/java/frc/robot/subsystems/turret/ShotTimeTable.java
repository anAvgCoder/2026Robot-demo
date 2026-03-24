package frc.robot.subsystems.turret;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShotTimeTable {
  private static final InterpolatingTreeMap<Double, Double> table =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), (a, b, t) -> a + (b - a) * t);

  static {
    // key = distanceMeters, value = flightTimeSeconds
    table.put(0.00, 1.22);
    table.put(1.55, 1.22);
    table.put(1.86, 1.22);
    table.put(2.14, 1.25);
    table.put(2.45, 1.25);
    table.put(2.73, 1.25);
    table.put(3.06, 1.27);
    table.put(3.42, 1.27);
    table.put(3.74, 1.27);
    table.put(4.05, 1.32);
    table.put(4.44, 1.33);
    table.put(4.90, 1.35);
    table.put(5.34, 1.37);
    table.put(7.50, 1.5);
    table.put(10.00, 1.60);
    table.put(15.00, 1.70);

    // table.put(0.00, .75);
    // table.put(1.55, .75);
    // table.put(1.86, .75);
    // table.put(2.14, .75);
    // table.put(2.45, .75);
    // table.put(2.73, .75);
    // table.put(3.06, .75);
    // table.put(3.42, .75);
    // table.put(3.74, .75);
    // table.put(4.05, .75);
    // table.put(4.44, .75);
    // table.put(4.90, .75);
    // table.put(5.34, .75);
    // table.put(7.50, .75);
    // table.put(10.00, .75);
    // table.put(15.00, .75);
  }

  public static double getFlightTimeSeconds(double distanceMeters) {
    return table.get(distanceMeters);
  }
}
