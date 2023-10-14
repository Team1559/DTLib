package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.subsystem.DTSubsystem;

import java.util.function.BooleanSupplier;

/**
 * A command that accepts functions for each of the basic command methods.
 * Useful for inline definitions of commands - note, however, that if a command
 * is too complex, it is better practice to write a proper command class than to
 * inline it.
 */
public class DTFunctionalCommand extends DTCommandBase {
  private final Runnable        init;
  private final Runnable        execute;
  private final Runnable        end;
  private final Runnable        interrupt;
  private final BooleanSupplier isFinished;

  /**
   * Constructs a new DTFunctionalCommand.
   *
   * @param init
   *        the initialization function
   * @param execute
   *        the execution function
   * @param end
   *        the end function
   * @param interrupt
   *        the interrupt handler
   * @param isFinished
   *        the condition to end this command
   * @param requirements
   *        the subsystems required by this command
   *
   * @see DTCommand#initialize()
   * @see DTCommand#execute()
   * @see DTCommand#end()
   * @see DTCommand#interrupt()
   * @see DTCommand#isFinished()
   */
  public DTFunctionalCommand(Runnable init, Runnable execute, Runnable end, Runnable interrupt,
                             BooleanSupplier isFinished, DTSubsystem... requirements) {
    addRequirements(requirements);
    this.init = init;
    this.execute = execute;
    this.end = end;
    this.interrupt = interrupt;
    this.isFinished = isFinished;
  }

  @Override
  public void initialize() {
    init.run();
  }

  @Override
  public void execute() {
    execute.run();
  }

  @Override
  public void end() {
    end.run();
  }

  @Override
  public void interrupt() {
    interrupt.run();
  }

  @Override
  public boolean isFinished() {
    return isFinished.getAsBoolean();
  }
}
