package org.victorrobotics.dtlib.command.util;

public class DTInstantCommand extends DTFunctionalCommand {
  public DTInstantCommand(Runnable toRun) {
    super(toRun, () -> {
    }, () -> {
    }, () -> {
    }, () -> true);
  }
}
