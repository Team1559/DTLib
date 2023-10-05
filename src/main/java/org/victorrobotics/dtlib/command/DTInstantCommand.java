package org.victorrobotics.dtlib.command;

public class DTInstantCommand extends DTFunctionalCommand {
  public DTInstantCommand(Runnable toRun) {
    super(toRun, () -> {}, () -> {}, () -> {}, () -> true);
  }
}
