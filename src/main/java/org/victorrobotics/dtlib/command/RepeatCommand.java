package org.victorrobotics.dtlib.command;

/**
 * A command that runs another command repeatedly, restarting it when it ends,
 * until this command is interrupted. Command instances that are passed to it
 * cannot be added to any other groups, or scheduled individually.
 * <p>
 * The rules for command compositions apply: command instances that are passed
 * to it cannot be added to any other composition or scheduled individually, and
 * the composition requires all subsystems its components require.
 */
public class RepeatCommand extends TargetCommand {
  private boolean wasSuccessful;

  /**
   * Constructs a new DTRepeatCommand.
   *
   * @param target
   *        the command to repeat
   */
  public RepeatCommand(Command target) {
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
