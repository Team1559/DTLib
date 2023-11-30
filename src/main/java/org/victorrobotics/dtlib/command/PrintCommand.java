package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.log.LogWriter;

/**
 * A command that prints text to the console when run, finishing immediately.
 * Useful for debugging command logic.
 */
public class PrintCommand extends InstantCommand {
  /**
   * Constructs a new PrintCommand.
   *
   * @param message the text to be printed
   */
  public PrintCommand(String message) {
    super(() -> {
      if (!LogWriter.info(message)) {
        System.out.println(message);
      }
    });
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
