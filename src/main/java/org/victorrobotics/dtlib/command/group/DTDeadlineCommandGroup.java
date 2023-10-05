package org.victorrobotics.dtlib.command.group;

import org.victorrobotics.dtlib.command.DTCommand;
import org.victorrobotics.dtlib.command.DTCommandBase;
import org.victorrobotics.dtlib.command.DTCommandScheduler;
import org.victorrobotics.dtlib.exception.DTIllegalArgumentException;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

public class DTDeadlineCommandGroup extends DTCommandBase {
  private final Map<DTCommand, Boolean> commands;
  private final DTCommand               deadline;

  private boolean runsWhenDisabled;
  private boolean isFinished;
  private boolean isInterruptible;

  public DTDeadlineCommandGroup(DTCommand deadline, DTCommand... commands) {
    this.deadline = Objects.requireNonNull(deadline);
    this.commands = new HashMap<>(commands.length + 1);
    runsWhenDisabled = true;

    addCommands(deadline);
    addCommands(commands);
  }

  public void addCommands(DTCommand... commands) {
    if (!isFinished) {
      throw new IllegalStateException("Cannot add commands to a running composition");
    } else if (commands == null || commands.length == 0) {
      return;
    }

    DTCommandScheduler.registerComposed(commands);

    for (DTCommand command : commands) {
      if (!Collections.disjoint(command.getRequirements(), requirements)) {
        throw new DTIllegalArgumentException(command, "parallel commands may not share requirements");
      }
      this.commands.put(command, false);
      requirements.addAll(command.getRequirements());
      runsWhenDisabled &= command.runsWhenDisabled();
      isInterruptible |= command.isInterruptible();
    }
  }

  @Override
  public void initialize() {
    for (Map.Entry<DTCommand, Boolean> commandEntry : commands.entrySet()) {
      commandEntry.getKey()
                  .initialize();
      commandEntry.setValue(true);
    }
    isFinished = false;
  }

  @Override
  public void execute() {
    for (Map.Entry<DTCommand, Boolean> commandEntry : commands.entrySet()) {
      if (!commandEntry.getValue()) {
        continue;
      }
      DTCommand command = commandEntry.getKey();
      command.execute();
      if (command.isFinished()) {
        command.end();
        commandEntry.setValue(false);
        if (command == deadline) {
          isFinished = true;
        }
      }
    }
  }

  @Override
  public void end() {
    for (Map.Entry<DTCommand, Boolean> commandEntry : commands.entrySet()) {
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
