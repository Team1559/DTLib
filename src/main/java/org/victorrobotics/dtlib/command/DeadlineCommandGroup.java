package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.subsystem.Subsystem;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * A command composition that runs a set of commands in parallel, ending only
 * when a specific command (the "deadline") ends, and then interrupting all
 * other commands that are still running.
 * <p>
 * The rules for command compositions apply: command instances that are passed
 * to it cannot be added to any other composition or scheduled individually, and
 * the composition requires all subsystems its components require.
 */
public class DeadlineCommandGroup extends Command {
  private final Map<Command, Boolean> commands;
  private final Command               deadline;

  private boolean runsWhenDisabled = true;
  private boolean isInterruptible  = true;

  private boolean isFinished;

  /**
   * Creates a new DeadlineCommandGroup. The given commands (including the
   * deadline) will be executed in parallel. The composition will finish when
   * the deadline finishes, interrupting all other still-running commands. If
   * the composition is interrupted, only the commands still running will be
   * interrupted.
   *
   * @param deadline the command that determines when the composition ends
   * @param commands the additional command(s) to be executed
   */
  public DeadlineCommandGroup(Command deadline, Command... commands) {
    this.deadline = Objects.requireNonNull(deadline);
    this.commands = new LinkedHashMap<>();

    CommandScheduler.registerComposed(deadline);
    addRequirements(deadline.getRequirements());
    runsWhenDisabled &= deadline.runsWhenDisabled();
    isInterruptible &= deadline.isInterruptible();

    addCommands(commands);
  }

  /**
   * Adds additional commands to the composition, which will run parallel to
   * existing commands.
   *
   * @param commands the commands to add
   * @throws IllegalStateException if the composition is currently scheduled
   * @throws IllegalArgumentException if a given command is already part of
   *         another composition, or if commands share requirements
   */
  public void addCommands(Command... commands) {
    if (isScheduled()) {
      throw new IllegalStateException("Cannot add commands to a running composition");
    } else if (commands == null || commands.length == 0) return;

    CommandScheduler.registerComposed(commands);

    for (Command command : commands) {
      if (command == null) continue;

      Set<Subsystem> commandReqs = command.getRequirements();
      if (!Collections.disjoint(getRequirements(), commandReqs)) {
        throw new IllegalArgumentException("parallel commands may not share requirements");
      }
      addRequirements(commandReqs);

      this.commands.put(command, false);
      runsWhenDisabled &= command.runsWhenDisabled();
      isInterruptible &= command.isInterruptible();
    }
  }

  @Override
  public void initialize() {
    isFinished = false;
    deadline.initialize();
    for (Map.Entry<Command, Boolean> commandEntry : commands.entrySet()) {
      commandEntry.getKey()
                  .initialize();
      commandEntry.setValue(true);
    }
  }

  @Override
  public void execute() {
    deadline.execute();
    if (deadline.isFinished()) {
      deadline.end();
      isFinished = true;
    }

    for (Map.Entry<Command, Boolean> commandEntry : commands.entrySet()) {
      if (!commandEntry.getValue()) continue;

      Command command = commandEntry.getKey();
      command.execute();
      if (command.isFinished()) {
        command.end();
        commandEntry.setValue(false);
      }
    }
  }

  @Override
  public void interrupt() {
    deadline.interrupt();
    end();
  }

  @Override
  public void end() {
    for (Map.Entry<Command, Boolean> commandEntry : commands.entrySet()) {
      if (commandEntry.getValue()) {
        commandEntry.getKey()
                    .interrupt();
      }
    }
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

  @Override
  public boolean runsWhenDisabled() {
    return runsWhenDisabled;
  }

  @Override
  public boolean isInterruptible() {
    return isInterruptible;
  }
}
