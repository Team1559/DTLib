package org.victorrobotics.dtlib.command.util;

import org.victorrobotics.dtlib.command.DTCommandBase;

import java.util.function.BooleanSupplier;

public class DTWaitUntilCommand extends DTCommandBase {
  private final BooleanSupplier condition;

  public DTWaitUntilCommand(BooleanSupplier condition) {
    this.condition = condition;
  }

  @Override
  public boolean isFinished() {
    return condition.getAsBoolean();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}