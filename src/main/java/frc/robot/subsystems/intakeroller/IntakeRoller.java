package frc.robot.subsystems.intakeroller;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends SubsystemBase implements IntakeRollerIO {
  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

  private static final double JAM_CURRENT_THRESHOLD_AMPS = 20.0;
  private static final double JAM_CONFIRM_SECONDS = 0.25;
  private static final double JAM_REVERSE_SECONDS = 0.25;

  private enum RollerState {
    IDLE,
    INTAKING,
    JAM_REVERSING
  }

  private RollerState state = RollerState.IDLE;

  private final Timer jamConfirmTimer = new Timer();
  private boolean jamConfirmTimerRunning = false;

  private final Timer jamReverseTimer = new Timer();

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
  }

  public IntakeRollerIO getIO() {
    return this.io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRoller", inputs);

    switch (state) {
      case INTAKING:
        if (!inputs.paused) {
          handleJamDetection();
        }
        break;

      case JAM_REVERSING:
        if (jamReverseTimer.hasElapsed(JAM_REVERSE_SECONDS)) {
          jamReverseTimer.stop();
          state = RollerState.INTAKING;
          io.intake();
        }
        break;

      case IDLE:
      default:
        break;
    }

    Logger.recordOutput("IntakeRoller/State", state.toString());
    Logger.recordOutput("IntakeRoller/JamDetected", state == RollerState.JAM_REVERSING);
  }

  private void handleJamDetection() {
    if (inputs.supplyCurrent > JAM_CURRENT_THRESHOLD_AMPS) {
      if (!jamConfirmTimerRunning) {
        jamConfirmTimer.reset();
        jamConfirmTimer.start();
        jamConfirmTimerRunning = true;
      }

      if (jamConfirmTimer.hasElapsed(JAM_CONFIRM_SECONDS)) {
        state = RollerState.JAM_REVERSING;
        resetJamConfirmTimer();
        jamReverseTimer.reset();
        jamReverseTimer.start();
        io.outake();
      }
    } else {
      resetJamConfirmTimer();
    }
  }

  private void resetJamConfirmTimer() {
    jamConfirmTimer.stop();
    jamConfirmTimer.reset();
    jamConfirmTimerRunning = false;
  }

  @Override
  public void intake() {
    state = RollerState.INTAKING;
    resetJamConfirmTimer();
    io.intake();
  }

  @Override
  public void outake() {
    state = RollerState.IDLE;
    resetJamConfirmTimer();
    io.outake();
  }

  @Override
  public void stop() {
    state = RollerState.IDLE;
    resetJamConfirmTimer();
    io.stop();
  }

  @Override
  public void setPaused(boolean value) {
    io.setPaused(value);
  }

  public void pause() {
    io.setPaused(true);
  }

  public void resume() {
    io.setPaused(false);
  }
}
