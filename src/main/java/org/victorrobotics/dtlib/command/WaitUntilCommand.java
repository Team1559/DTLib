package org.victorrobotics.dtlib.command;

import java.util.function.BooleanSupplier;

/**
 * A command that waits for a certain condition to be met before finishing. This
 * does not pause the entire robot program, but rather only the command
 * composition it belongs to. Useful for delaying command execution or pausing
 * between actions.
 */
public class WaitUntilCommand extends Command {
  private final BooleanSupplier condition;

  /**
   * Constructs a new WaitUntilCommand.
   *
   * @param condition the condition to wait for
   */
  public WaitUntilCommand(BooleanSupplier condition) {
    this.condition = condition;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end() {}

  @Override
  public boolean isFinished() {
    return condition.getAsBoolean();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
