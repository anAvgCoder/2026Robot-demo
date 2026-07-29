package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Left flight stick (port 0): outtake, manual overrides, and red-alliance pose resets. */
public class LeftStick {
  public final Joystick joystick = new Joystick(0);

  public final JoystickButton outtakeButton = new JoystickButton(joystick, 1);
  public final JoystickButton button2 = new JoystickButton(joystick, 2);
  public final JoystickButton button3 = new JoystickButton(joystick, 3);
  public final JoystickButton button4 = new JoystickButton(joystick, 4);
  public final JoystickButton toggleAutoAimButton = new JoystickButton(joystick, 5);
  public final JoystickButton resetQuestPoseRedButton = new JoystickButton(joystick, 11);
  public final JoystickButton button13 = new JoystickButton(joystick, 13);
  public final JoystickButton button14 = new JoystickButton(joystick, 14);
}
