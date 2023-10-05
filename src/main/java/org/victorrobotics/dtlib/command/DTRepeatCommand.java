package org.victorrobotics.dtlib.command;

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
