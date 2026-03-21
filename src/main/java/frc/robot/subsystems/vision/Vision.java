package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  VisionIO io;

  public Vision(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.processVision();
  }
}
