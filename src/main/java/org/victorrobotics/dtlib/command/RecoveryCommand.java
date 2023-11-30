package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.log.DTLog;
import org.victorrobotics.dtlib.log.LogWriter;

/**
 * A command that runs another command, but catches runtime exceptions instead
 * of propogating them back to the scheduler. If an exception is thrown, the
 * command will interrupt itself and {@link #wasSuccessful()} will return false.
 * <p>
 * The rules for command compositions apply: command instances that are passed
 * to it cannot be added to any other composition or scheduled individually, and
 * the composition requires all subsystems its components require.
 */
public class RecoveryCommand extends TargetCommand {
  private boolean threwException;

  /**
   * Constructs a new RecoveryCommand.
   *
   * @param target the command to call
   */
  public RecoveryCommand(Command target) {
    super(target);
  }

  @Override
  public void initialize() {
    threwException = false;

    try {
      target.initialize();
    } catch (RuntimeException e) {
      LogWriter.logException(e, DTLog.Level.DEBUG);
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
      LogWriter.logException(e, DTLog.Level.DEBUG);
      threwException = true;
    }
  }

  @Override
  public void end() {
    if (threwException) {
      try {
        target.interrupt();
      } catch (RuntimeException e) {
        LogWriter.logException(e, DTLog.Level.DEBUG);
      }
      return;
    }

    try {
      target.end();
    } catch (RuntimeException e) {
      LogWriter.logException(e, DTLog.Level.DEBUG);
      threwException = true;
    }
  }

  @Override
  public void interrupt() {
    try {
      target.interrupt();
    } catch (RuntimeException e) {
      LogWriter.logException(e, DTLog.Level.DEBUG);
      threwException = true;
    }
  }

  @Override
  public boolean isFinished() {
    try {
      return threwException || target.isFinished();
    } catch (RuntimeException e) {
      LogWriter.logException(e, DTLog.Level.DEBUG);
      threwException = true;
      return true;
    }
  }

  @Override
  public boolean wasSuccessful() {
    try {
      return !threwException && target.wasSuccessful();
    } catch (RuntimeException e) {
      LogWriter.logException(e, DTLog.Level.DEBUG);
      threwException = true;
      return false;
    }
  }
}
