package org.victorrobotics.dtlib.command;

/**
 * A command that runs a single action and finishes immediately; it will begin
 * execution and end on the same iteration of the scheduler.
 */
public class DTInstantCommand extends DTFunctionalCommand {
  /**
   * Constructs a new DTInstantCommand.
   *
   * @param toRun
   *        the action to be run
   */
  public DTInstantCommand(Runnable toRun) {
    super(toRun, () -> {}, () -> {}, () -> {}, () -> true);
  }
}
