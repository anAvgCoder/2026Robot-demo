package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Right flight stick (port 1): intake, drive speed modes, and blue-alliance pose resets. */
public class RightStick {
  public final Joystick joystick = new Joystick(1);

  public final JoystickButton intakeButton = new JoystickButton(joystick, 1);
  public final JoystickButton lockHeadingButton = new JoystickButton(joystick, 2);
  public final JoystickButton fullSpeedButton = new JoystickButton(joystick, 3);
  public final JoystickButton intakeSpeedButton = new JoystickButton(joystick, 4);
  public final JoystickButton resetQuestPoseBlueHubButton = new JoystickButton(joystick, 5);
  public final JoystickButton button6 = new JoystickButton(joystick, 6);
  public final JoystickButton resetPoseBlueHubButton = new JoystickButton(joystick, 7);
  public final JoystickButton button8 = new JoystickButton(joystick, 8);
  public final JoystickButton button9 = new JoystickButton(joystick, 9);
  public final JoystickButton button10 = new JoystickButton(joystick, 10);
  public final JoystickButton resetQuestPoseBlueButton = new JoystickButton(joystick, 11);
}
