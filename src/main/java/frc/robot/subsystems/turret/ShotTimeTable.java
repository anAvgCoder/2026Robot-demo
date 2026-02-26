package frc.robot.subsystems.turret;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShotTimeTable {
  private static final InterpolatingTreeMap<Double, Double> table =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), (a, b, t) -> a + (b - a) * t);

  public ShotTimeTable() {
    // key = distanceMeters, value = flightTimeSeconds
    table.put(1.0, 0.25);
    table.put(2.0, 0.35);
    table.put(3.5, 0.50);
  }

  public static double getFlightTimeSeconds(double distanceMeters) {
    return table.get(distanceMeters);
  }
}