package org.victorrobotics.frc.dtlib.command.util;

public class DTInstantCommand extends DTFunctionalCommand {
  public DTInstantCommand(Runnable toRun) {
    super(toRun, () -> {
    }, () -> {
    }, () -> {
    }, () -> true);
  }
}
