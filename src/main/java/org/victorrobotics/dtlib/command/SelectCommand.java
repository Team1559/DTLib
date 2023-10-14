package org.victorrobotics.dtlib.command;

import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

/**
 * A command that executes a command obtained from a map. Upon initialization,
 * the command corresponding to the selector's return value will begin
 * execution.
 * <p>
 * The rules for command compositions apply: command instances that are passed
 * to it cannot be added to any other composition or scheduled individually, and
 * the composition requires all subsystems its components require.
 */
public class SelectCommand<T> extends CommandBase {
  private final Map<T, Command> commandMap;
  private final Supplier<T>       selector;

  private Command selectedCommand;

  /**
   * Constructs a new DTSelectCommand.
   *
   * @param selector the key supplier that decides which command to execute
   * @param entries the key and command pairs to select from
   */
  @SafeVarargs
  public SelectCommand(Supplier<T> selector, Map.Entry<T, Command>... entries) {
    this(selector, Map.ofEntries(entries));
  }

  /**
   * Constructs a new DTSelectCommand.
   *
   * @param selector the key supplier that decides which command to execute
   * @param commands the map of commands to select from
   */
  public SelectCommand(Supplier<T> selector, Map<T, Command> commands) {
    this.commandMap = Objects.requireNonNull(commands);
    this.selector = Objects.requireNonNull(selector);
    CommandScheduler.registerComposed(commands.values());

    commands.values()
            .forEach(command -> addRequirements(command.getRequirements()));
  }

  @Override
  public void initialize() {
    T key = selector.get();
    selectedCommand = commandMap.get(key);
    if (selectedCommand == null) {
      selectedCommand = new NullCommand();
    }

    selectedCommand.initialize();
  }

  @Override
  public void execute() {
    selectedCommand.execute();
  }

  @Override
  public void end() {
    selectedCommand.end();
  }

  @Override
  public void interrupt() {
    selectedCommand.interrupt();
  }

  @Override
  public boolean isFinished() {
    return selectedCommand.isFinished();
  }

  @Override
  public boolean runsWhenDisabled() {
    return selectedCommand.runsWhenDisabled();
  }

  @Override
  public boolean isInterruptible() {
    return selectedCommand.isInterruptible();
  }
}
