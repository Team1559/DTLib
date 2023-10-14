package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.subsystem.DTSubsystem;

/**
 * A command that continually runs a function every iteration of the scheduler
 * until it is cancelled.
 */
public class RunCommand extends FunctionalCommand {
  /**
   * Constructs a new DTRunCommand.
   *
   * @param toRun
   *        the function to run on loop
   * @param requirements
   *        the required subsystems
   */
  public RunCommand(Runnable toRun, DTSubsystem... requirements) {
    super(() -> {}, toRun, () -> {}, () -> {}, () -> false, requirements);
  }
}
