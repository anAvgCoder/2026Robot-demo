package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Button panel (port 2): turret presets, manual nudges, pause/resume, and pose-source toggle. */
public class OperatorPanel {
  public final Joystick joystick = new Joystick(2);

  public final JoystickButton rotateToNeg60Button = new JoystickButton(joystick, 1);
  public final JoystickButton rotateTo0Button = new JoystickButton(joystick, 2);
  public final JoystickButton rotateTo60Button = new JoystickButton(joystick, 3);
  public final JoystickButton rotateToNeg90Button = new JoystickButton(joystick, 4);
  public final JoystickButton pauseAllButton = new JoystickButton(joystick, 5);
  public final JoystickButton rotateTo90Button = new JoystickButton(joystick, 6);
  public final JoystickButton switchToCameraButton = new JoystickButton(joystick, 7);
  public final JoystickButton switchToQuestButton = new JoystickButton(joystick, 8);
  public final JoystickButton hoodDownButton = new JoystickButton(joystick, 9);
  public final JoystickButton hoodUpButton = new JoystickButton(joystick, 11);
  public final JoystickButton turretNudgePositiveButton = new JoystickButton(joystick, 12);
  public final JoystickButton turretNudgeNegativeButton = new JoystickButton(joystick, 13);
  public final JoystickButton shooterNudgeDownButton = new JoystickButton(joystick, 14);
  public final JoystickButton shooterNudgeUpButton = new JoystickButton(joystick, 15);
}
