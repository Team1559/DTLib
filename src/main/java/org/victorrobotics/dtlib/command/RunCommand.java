package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.subsystem.Subsystem;

/**
 * A command that continually runs a function every iteration of the scheduler
 * until it is cancelled.
 */
public class RunCommand extends FunctionalCommand {
  /**
   * Constructs a new RunCommand.
   *
   * @param toRun the function to run on loop
   * @param requirements the required subsystems
   */
  public RunCommand(Runnable toRun, Subsystem... requirements) {
    super(() -> {}, toRun, () -> {}, () -> {}, () -> false, requirements);
  }
}
