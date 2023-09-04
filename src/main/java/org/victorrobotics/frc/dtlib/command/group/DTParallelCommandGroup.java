package org.victorrobotics.frc.dtlib.command.group;

import org.victorrobotics.frc.dtlib.DTSubsystem;
import org.victorrobotics.frc.dtlib.command.DTCommand;
import org.victorrobotics.frc.dtlib.command.DTCommandBase;
import org.victorrobotics.frc.dtlib.command.DTCommandScheduler;
import org.victorrobotics.frc.dtlib.exception.DTIllegalArgumentException;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

public class DTParallelCommandGroup extends DTCommandBase {
  private final Map<DTCommand, Boolean> parallelCommands;

  private boolean runsWhenDisabled; // if all run when disabled
  private boolean isInterruptible;  // if one is interruptible
  private boolean isRunning;
  private boolean success;

  public DTParallelCommandGroup(DTCommand... commands) {
    parallelCommands = new HashMap<>(commands.length);
    runsWhenDisabled = true;
    isInterruptible = false;
    success = true;
    addCommands(commands);
  }

  public void addCommands(DTCommand... commands) {
    if (isRunning) {
      throw new IllegalStateException("Cannot add commands to a running composition");
    } else if (commands == null || commands.length == 0) {
      return;
    }
    DTCommandScheduler.registerComposedCommands(commands);

    for (DTCommand command : commands) {
      Set<DTSubsystem> commandReqs = command.getRequirements();
      if (!Collections.disjoint(requirements, commandReqs)) {
        throw new DTIllegalArgumentException(command, "parallel commands may not share requirements");
      }
      parallelCommands.put(command, false);
      requirements.addAll(commandReqs);
      runsWhenDisabled &= command.runsWhenDisabled();
      isInterruptible |= command.isInterruptible();
    }
  }

  public void addCommand(DTCommand command) {
    if (isRunning) {
      throw new IllegalStateException("Cannot add commands to a running composition");
    }
    DTCommandScheduler.registerComposedCommands(command);

    Set<DTSubsystem> commandReqs = command.getRequirements();
    if (!Collections.disjoint(requirements, commandReqs)) {
      throw new DTIllegalArgumentException(command, "parallel commands may not share requirements");
    }
    parallelCommands.put(command, false);
    requirements.addAll(commandReqs);
    runsWhenDisabled &= command.runsWhenDisabled();
    isInterruptible |= command.isInterruptible();
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
      if (!commandEntry.getValue()) {
        // Already complete
        continue;
      }
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
