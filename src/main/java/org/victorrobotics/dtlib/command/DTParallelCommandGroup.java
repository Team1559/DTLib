package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.exception.DTIllegalArgumentException;
import org.victorrobotics.dtlib.subsystem.DTSubsystem;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

/**
 * A command composition that runs a set of commands in parallel, ending when
 * the last command ends.
 * <p>
 * The rules for command compositions apply: command instances that are passed
 * to it cannot be added to any other composition or scheduled individually, and
 * the composition requires all subsystems its components require.
 */
public class DTParallelCommandGroup extends DTCommandBase {
  private final Map<DTCommand, Boolean> parallelCommands;

  private boolean runsWhenDisabled = true;
  private boolean isInterruptible  = true;

  private boolean isRunning;
  private boolean success;

  /**
   * Constructs a DTParallelCommandGroup
   *
   * @param commands
   *        the commands to run in parallel
   */
  public DTParallelCommandGroup(DTCommand... commands) {
    parallelCommands = new LinkedHashMap<>();
    success = true;
    addCommands(commands);
  }

  /**
   * Adds additional commands to the composition, which will run parallel to
   * current commands.
   *
   * @param commands
   *        the commands to add
   *
   * @throws IllegalStateException
   *         if the composition is currently scheduled
   * @throws DTIllegalArgumentException
   *         if a given command is already part of another composition, or if
   *         commands share requirements
   */
  public void addCommands(DTCommand... commands) {
    if (isRunning) {
      throw new IllegalStateException("Cannot add commands to a running composition");
    } else if (commands == null || commands.length == 0) return;

    DTCommandScheduler.registerComposed(commands);

    for (DTCommand command : commands) {
      if (command == null) continue;

      Set<DTSubsystem> commandReqs = command.getRequirements();
      if (!Collections.disjoint(getRequirements(), commandReqs)) {
        throw new DTIllegalArgumentException(command,
                                             "parallel commands may not share requirements");
      }
      addRequirements(commandReqs);

      parallelCommands.put(command, false);
      runsWhenDisabled &= command.runsWhenDisabled();
      isInterruptible &= command.isInterruptible();
    }
  }

  @Override
  public void initialize() {
    isRunning = true;
    success = true;
    for (Entry<DTCommand, Boolean> commandEntry : parallelCommands.entrySet()) {
      commandEntry.getKey()
                  .initialize();
      commandEntry.setValue(true);
    }
  }

  @Override
  public void execute() {
    for (Entry<DTCommand, Boolean> commandEntry : parallelCommands.entrySet()) {
      if (!commandEntry.getValue()) continue;

      DTCommand command = commandEntry.getKey();
      command.execute();
      if (command.isFinished()) {
        command.end();
        success &= command.wasSuccessful();
        commandEntry.setValue(false);
      }
    }
  }

  @Override
  public void end() {
    isRunning = false;
  }

  @Override
  public void interrupt() {
    isRunning = false;
    for (Entry<DTCommand, Boolean> commandEntry : parallelCommands.entrySet()) {
      if (commandEntry.getValue()) {
        DTCommand command = commandEntry.getKey();
        command.interrupt();
        success &= command.wasSuccessful();
      }
    }
  }

  @Override
  public boolean isFinished() {
    return !parallelCommands.containsValue(true);
  }

  @Override
  public boolean runsWhenDisabled() {
    return runsWhenDisabled;
  }

  @Override
  public boolean isInterruptible() {
    return isInterruptible;
  }

  @Override
  public boolean wasSuccessful() {
    return success;
  }

  @Override
  public DTParallelCommandGroup alongWith(DTCommand... parallel) {
    addCommands(parallel);
    return this;
  }
}
