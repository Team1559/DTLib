package org.victorrobotics.frc.dtlib.command.util;

import org.victorrobotics.frc.dtlib.command.DTCommand;

public class DTRepeatCommand extends DTTargetCommand {
  private boolean wasSuccessful;

  public DTRepeatCommand(DTCommand target) {
    super(target);
  }

  @Override
  public void execute() {
    target.execute();
    if (target.isFinished()) {
      target.end();
      wasSuccessful = target.wasSuccessful();
      target.initialize();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean wasSuccessful() {
    return wasSuccessful;
  }
}
