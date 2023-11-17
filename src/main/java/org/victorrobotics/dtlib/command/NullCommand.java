package org.victorrobotics.dtlib.command;

/**
 * A command that explicitly does nothing and finishes immediately. Useful for
 * temporarily disabling behavior or selective schedulling.
 */
public class NullCommand extends InstantCommand {
  /**
   * Constructs a new NullCommand.
   */
  public NullCommand() {
    super(() -> {});
  }
}
