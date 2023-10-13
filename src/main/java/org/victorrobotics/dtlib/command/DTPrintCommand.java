package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.log.DTLogWriter;

/**
 * A command that prints text to the console when run, finishing immediately.
 * Useful for debugging command logic.
 */
public class DTPrintCommand extends DTInstantCommand {
  /**
   * Constructs a new DTPrintCommand.
   *
   * @param message
   *        the text to be printed
   */
  public DTPrintCommand(String message) {
    super(() -> {
      if (!DTLogWriter.info(message)) {
        System.out.println(message);
      }
    });
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
