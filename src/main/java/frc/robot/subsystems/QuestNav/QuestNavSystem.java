package frc.robot.subsystems.QuestNav;

import static frc.robot.subsystems.QuestNav.QuestNavSystemConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class QuestNavSystem extends SubsystemBase {
  private final QuestNavSystemIO io;

  public QuestNavSystem(QuestNavSystemIO io) {
    this.io = io;

    // set initial reference point
    io.setQuestPose(defaultQuestPose);
  }

  public QuestNavSystemIO getIO() {
    return this.io;
  }

  @Override
  public void periodic() {
    io.runQuestCommand();

    io.updateQuestStatus();
  }
}
