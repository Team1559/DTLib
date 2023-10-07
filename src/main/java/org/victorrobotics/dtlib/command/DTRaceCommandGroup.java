package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.exception.DTIllegalArgumentException;
import org.victorrobotics.dtlib.subsystem.DTSubsystem;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

/**
 * A composition that runs a set of commands in parallel, ending when any one of
 * the commands ends and interrupting all the others.
 * <p>
 * The rules for command compositions apply: command instances that are passed
 * to it cannot be added to any other composition or scheduled individually, and
 * the composition requires all subsystems its components require.
 */
public class DTRaceCommandGroup extends DTCommandBase {
  private final Map<DTCommand, Boolean> raceCommands;

  private boolean runsWhenDisabled = true;
  private boolean isInterruptible  = true;

  private boolean isFinished;
  private boolean success;

  /**
   * Constructs a DTRaceCommandGroup
   *
   * @param commands
   *        the commands to race in parallel
   */
  public DTRaceCommandGroup(DTCommand... commands) {
    raceCommands = new HashMap<>(commands.length);
    isFinished = true;
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
    if (!isFinished) {
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

      raceCommands.put(command, false);
      runsWhenDisabled &= command.runsWhenDisabled();
      isInterruptible &= command.isInterruptible();
    }
  }

  @Override
  public void initialize() {
    isFinished = false;
    success = true;
    for (DTCommand command : raceCommands.keySet()) {
      command.initialize();
    }
  }

  @Override
  public void execute() {
    for (Entry<DTCommand, Boolean> commandEntry : raceCommands.entrySet()) {
      DTCommand command = commandEntry.getKey();
      command.execute();
      if (command.isFinished()) {
        command.end();
        success &= command.wasSuccessful();
        commandEntry.setValue(true);
        isFinished = true;
      }
    }
  }

  @Override
  public void end() {
    for (Entry<DTCommand, Boolean> commandEntry : raceCommands.entrySet()) {
      if (commandEntry.getValue()) continue;

      DTCommand command = commandEntry.getKey();
      command.interrupt();
      success &= command.wasSuccessful();
    }
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

  @Override
  public boolean wasSuccessful() {
    return success;
  }

  @Override
  public boolean isInterruptible() {
    return isInterruptible;
  }

  @Override
  public boolean runsWhenDisabled() {
    return runsWhenDisabled;
  }

  @Override
  public DTRaceCommandGroup raceWith(DTCommand... parallel) {
    addCommands(parallel);
    return this;
  }
}
