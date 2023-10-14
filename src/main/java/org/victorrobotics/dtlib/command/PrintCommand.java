package org.victorrobotics.dtlib.command;

/**
 * A command that prints text to the console when run, finishing immediately.
 * Useful for debugging command logic.
 */
public class PrintCommand extends InstantCommand {
  /**
   * Constructs a new DTPrintCommand.
   *
   * @param message
   *        the text to be printed
   */
  public PrintCommand(String message) {
    super(() -> System.out.println(message));
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
