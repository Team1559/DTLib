package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.DTRobot;

/**
 * A command that waits for a certain period of time before finishing. This does
 * not pause the entire robot program, but rather only the command composition
 * it belongs to. Useful for delaying command execution or pausing between
 * actions.
 */
public class WaitCommand extends CommandBase {
  private final long duration;
  private long       endTime;

  /**
   * Constructs a new DTWaitCommand.
   *
   * @param duration
   *        the time to wait (in seconds)
   */
  public WaitCommand(double duration) {
    this.duration = (long) (duration * 1e6);
  }

  @Override
  public void initialize() {
    endTime = DTRobot.currentTimeMicros() + duration;
  }

  @Override
  public boolean isFinished() {
    return DTRobot.currentTimeMicros() >= endTime;
  }

  @Override
  public void end() {
    endTime = -1;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
