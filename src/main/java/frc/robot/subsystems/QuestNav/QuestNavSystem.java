package frc.robot.subsystems.questnav;

import static frc.robot.subsystems.questnav.QuestNavSystemConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class QuestNavSystem extends SubsystemBase {
  private final QuestNavSystemIO io;

  public QuestNavSystem(QuestNavSystemIO io) {
    this.io = io;

    // set initial reference point
    io.setQuestPose(QuestNavSystemConstants.ROBOT_TO_QUEST_BLUE);
  }

  public QuestNavSystemIO getIO() {
    return this.io;
  }
}
