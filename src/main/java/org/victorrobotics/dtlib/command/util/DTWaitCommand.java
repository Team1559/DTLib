package org.victorrobotics.dtlib.command.util;

import org.victorrobotics.dtlib.DTRobot;
import org.victorrobotics.dtlib.command.DTCommandBase;

public class DTWaitCommand extends DTCommandBase {
  private final long duration;
  private long       endTime;

  public DTWaitCommand(double duration) {
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
