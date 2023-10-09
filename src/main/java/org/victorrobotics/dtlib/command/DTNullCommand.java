package org.victorrobotics.dtlib.command;

/**
 * A command that explicitly does nothing and finishes immediately. Useful for
 * temporarily disabling behavior or selective schedulling.
 */
public class DTNullCommand extends DTInstantCommand {
  /**
   * Constructs a new DTNullCommand.
   */
  public DTNullCommand() {
    super(() -> {});
  }
}
