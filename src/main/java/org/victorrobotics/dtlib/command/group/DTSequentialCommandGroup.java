package org.victorrobotics.dtlib.command.group;

import org.victorrobotics.dtlib.command.DTCommand;
import org.victorrobotics.dtlib.command.DTCommandBase;
import org.victorrobotics.dtlib.command.DTCommandScheduler;

import java.util.ArrayList;
import java.util.List;

public class DTSequentialCommandGroup extends DTCommandBase {
  private final List<DTCommand> sequentialCommands;
  private int                   cmdIndex;
  private boolean               runsWhenDisabled;
  private boolean               isInterruptible;
  private boolean               success;

  public DTSequentialCommandGroup(DTCommand... commands) {
    sequentialCommands = new ArrayList<>();
    cmdIndex = -1;
    addCommands(commands);
  }

  public void addCommands(DTCommand... commands) {
    addCommands(sequentialCommands.size(), commands);
  }

  public void addCommands(int index, DTCommand... commands) {
    if (cmdIndex != -1) {
      throw new IllegalStateException("Cannot add commands to a running composition");
    } else if (commands == null || commands.length == 0) {
      return;
    }
    DTCommandScheduler.registerComposedCommands(commands);

    sequentialCommands.addAll(index, List.of(commands));
    for (DTCommand command : commands) {
      requirements.addAll(command.getRequirements());
      runsWhenDisabled &= command.runsWhenDisabled();
      isInterruptible |= command.isInterruptible();
    }
  }

  @Override
  public void initialize() {
    cmdIndex = 0;
    if (!sequentialCommands.isEmpty()) {
      sequentialCommands.get(0)
                        .initialize();
    }
  }

  @Override
  public void execute() {
    if (sequentialCommands.isEmpty()) {
      return;
    }

    DTCommand current = sequentialCommands.get(cmdIndex);
    current.execute();

    // Multiple commands can execute per loop cycle if they all finish
    while (current.isFinished()) {
      current.end();
      success &= current.wasSuccessful();
      cmdIndex++;
      if (cmdIndex == sequentialCommands.size()) {
        return;
      }
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
    if (!sequentialCommands.isEmpty() && cmdIndex > -1 && cmdIndex < sequentialCommands.size()) {
      DTCommand command = sequentialCommands.get(cmdIndex);
      command.interrupt();
      success &= command.wasSuccessful();
    }
    success &= cmdIndex >= sequentialCommands.size() - 1;
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
    return isInterruptible;
  }

  @Override
  public boolean wasSuccessful() {
    return success;
  }

  @Override
  public DTSequentialCommandGroup beforeStarting(DTCommand before) {
    addCommands(0, before);
    return this;
  }

  @Override
  public DTSequentialCommandGroup andThen(DTCommand... next) {
    addCommands(next);
    return this;
  }
}
