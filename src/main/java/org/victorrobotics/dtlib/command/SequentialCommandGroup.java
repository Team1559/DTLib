package org.victorrobotics.dtlib.command;

import java.util.ArrayList;
import java.util.List;

/**
 * A command composition that runs a list of commands individually in sequence.
 * <p>
 * The rules for command compositions apply: command instances that are passed
 * to it cannot be added to any other composition or scheduled individually, and
 * the composition requires all subsystems its components require.
 */
public class SequentialCommandGroup extends Command {
  private final List<Command> sequentialCommands;

  private boolean runsWhenDisabled = true;

  private int     cmdIndex;
  private boolean success;

  /**
   * Constructs a SequentialCommandGroup
   *
   * @param commands the commands to run, in sequence
   */
  public SequentialCommandGroup(Command... commands) {
    sequentialCommands = new ArrayList<>();
    cmdIndex = -1;
    addCommands(commands);
  }

  /**
   * Adds additional commands to the composition, which will run after existing
   * commands.
   *
   * @param commands the commands to add
   * @throws IllegalStateException if the composition is currently scheduled
   * @throws IllegalArgumentException if a given command is already part of
   *         another composition
   */
  public void addCommands(Command... commands) {
    if (isScheduled()) {
      throw new IllegalStateException("Cannot add commands to a running composition");
    } else if (commands == null || commands.length == 0) return;

    CommandScheduler.registerComposed(commands);

    for (Command command : commands) {
      if (command == null) continue;

      sequentialCommands.add(command);
      addRequirements(command.getRequirements());
      runsWhenDisabled &= command.runsWhenDisabled();
    }
  }

  @Override
  public void initialize() {
    cmdIndex = 0;
    success = true;
    if (!sequentialCommands.isEmpty()) {
      sequentialCommands.get(0)
                        .initialize();
    }
  }

  @Override
  public void execute() {
    if (sequentialCommands.isEmpty()) return;

    Command current = sequentialCommands.get(cmdIndex);
    current.execute();

    // Multiple commands can execute per loop cycle if they all finish
    while (current.isFinished()) {
      current.end();
      success &= current.wasSuccessful();
      cmdIndex++;
      if (cmdIndex == sequentialCommands.size()) return;

      current = sequentialCommands.get(cmdIndex);
      current.initialize();
      current.execute();
    }
  }

  @Override
  public void end() {
    cmdIndex = -1;
  }

  @Override
  public void interrupt() {
    if (cmdIndex >= 0 && cmdIndex < sequentialCommands.size()) {
      sequentialCommands.get(cmdIndex)
                        .interrupt();
    }
    success = false;
    cmdIndex = -1;
  }

  @Override
  public boolean isFinished() {
    return cmdIndex == sequentialCommands.size();
  }

  @Override
  public boolean runsWhenDisabled() {
    return runsWhenDisabled;
  }

  @Override
  public boolean isInterruptible() {
    return cmdIndex == -1 || sequentialCommands.get(cmdIndex)
                                               .isInterruptible();
  }

  @Override
  public boolean wasSuccessful() {
    return success && (cmdIndex == -1 || cmdIndex >= sequentialCommands.size());
  }

  @Override
  public SequentialCommandGroup beforeStarting(Command... before) {
    if (cmdIndex != -1) {
      throw new IllegalStateException("Cannot add commands to a running composition");
    } else if (before == null || before.length == 0) return this;

    CommandScheduler.registerComposed(before);

    int index = 0;
    for (Command command : before) {
      if (command == null) continue;

      sequentialCommands.add(index, command);
      index++;

      addRequirements(command.getRequirements());
      runsWhenDisabled &= command.runsWhenDisabled();
    }
    return this;
  }

  @Override
  public SequentialCommandGroup andThen(Command... next) {
    addCommands(next);
    return this;
  }
}
