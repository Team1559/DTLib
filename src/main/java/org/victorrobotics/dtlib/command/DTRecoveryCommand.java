package org.victorrobotics.dtlib.command;

public class DTRecoveryCommand extends DTTargetCommand {
  private boolean threwException;

  public DTRecoveryCommand(DTCommand target) {
    super(target);
  }

  @Override
  public void initialize() {
    threwException = false;

    try {
      target.initialize();
    } catch (RuntimeException e) {
      threwException = true;
    }
  }

  @Override
  public void execute() {
    if (threwException) {
      return;
    }

    try {
      target.execute();
    } catch (RuntimeException e) {
      threwException = true;
    }
  }

  @Override
  public void end() {
    try {
      target.end();
    } catch (RuntimeException e) {
      threwException = true;
    }
  }

  @Override
  public void interrupt() {
    try {
      target.interrupt();
    } catch (RuntimeException e) {
      threwException = true;
    }
  }

  @Override
  public boolean isFinished() {
    try {
      return threwException || target.isFinished();
    } catch (RuntimeException e) {
      threwException = true;
      return true;
    }
  }

  @Override
  public boolean wasSuccessful() {
    try {
      return !threwException && target.wasSuccessful();
    } catch (RuntimeException e) {
      threwException = true;
      return false;
    }
  }
}
