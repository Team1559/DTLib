package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.exception.DTIllegalArgumentException;

import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * A command composition that runs one of two commands, depending on the given
 * condition upon initialization.
 * <p>
 * The rules for command compositions apply: command instances that are passed
 * to it cannot be added to any other composition or scheduled individually, and
 * the composition requires all subsystems its components require.
 */
public class ConditionalCommand extends CommandBase {
  private final Command       trueCommand;
  private final Command       falseCommand;
  private final BooleanSupplier condition;

  private Command activeCommand;

  /**
   * Creates a new DTConditionalCommand
   *
   * @param onTrue
   *        the command to run if the condition is true
   * @param onFalse
   *        the command to run if the condition is false
   * @param condition
   *        the condition to determine which command to run
   *
   * @throws DTIllegalArgumentException
   *         if either command is part of another composition
   * @throws NullPointerException
   *         if the condition is null
   */
  public ConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
    trueCommand = onTrue != null ? onTrue : new NullCommand();
    falseCommand = onFalse != null ? onFalse : new NullCommand();
    this.condition = Objects.requireNonNull(condition);

    addRequirements(trueCommand.getRequirements());
    addRequirements(falseCommand.getRequirements());
    CommandScheduler.registerComposed(onTrue, onFalse);

    activeCommand = falseCommand;
  }

  @Override
  public void initialize() {
    activeCommand = condition.getAsBoolean() ? trueCommand : falseCommand;
    activeCommand.initialize();
  }

  @Override
  public void execute() {
    activeCommand.execute();
  }

  @Override
  public void end() {
    activeCommand.end();
  }

  @Override
  public void interrupt() {
    activeCommand.interrupt();
  }

  @Override
  public boolean isFinished() {
    return activeCommand.isFinished();
  }

  @Override
  public boolean wasSuccessful() {
    return activeCommand.wasSuccessful();
  }
}
