package org.victorrobotics.dtlib.command;

import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

public class DTSelectCommand<T> extends DTCommandBase {
  private final Map<T, DTCommand> commandMap;
  private final Supplier<T>       selector;

  private DTCommand selectedCommand;
  private boolean   runsWhenDisabled;
  private boolean   isInterruptible;

  public DTSelectCommand(Supplier<T> selector, Map<T, DTCommand> commands) {
    this.commandMap = Objects.requireNonNull(commands);
    this.selector = Objects.requireNonNull(selector);
    DTCommandScheduler.registerComposed(commands.values());

    boolean runsWhenDisabled = true;
    boolean isInterruptible = false;
    for (DTCommand command : commands.values()) {
      addRequirements(command.getRequirements());
      runsWhenDisabled &= command.runsWhenDisabled();
      isInterruptible |= command.isInterruptible();
    }
    this.runsWhenDisabled = runsWhenDisabled;
    this.isInterruptible = isInterruptible;
  }

  @Override
  public void initialize() {
    T key = selector.get();
    selectedCommand = commandMap.get(key);
    if (selectedCommand == null) {
      selectedCommand = new DTNullCommand();
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
  public boolean isFinished() { return selectedCommand.isFinished(); }

  @Override
  public boolean runsWhenDisabled() {
    return runsWhenDisabled;
  }

  @Override
  public boolean isInterruptible() { return isInterruptible; }

  @SafeVarargs
  public static <T> DTSelectCommand<T> of(Supplier<T> selector,
                                          Map.Entry<T, DTCommand>... entries) {
    return new DTSelectCommand<>(selector, Map.ofEntries(entries));
  }
}
