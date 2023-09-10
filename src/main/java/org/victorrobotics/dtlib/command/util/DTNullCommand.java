package org.victorrobotics.dtlib.command.util;

// Explicitly does nothing when called (cleaner code)
public class DTNullCommand extends DTInstantCommand {
  public DTNullCommand() {
    super(() -> {
    });
  }
}
