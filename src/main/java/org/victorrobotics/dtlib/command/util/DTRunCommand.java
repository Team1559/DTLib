package org.victorrobotics.dtlib.command.util;

import org.victorrobotics.dtlib.DTSubsystem;

public class DTRunCommand extends DTFunctionalCommand {
  public DTRunCommand(Runnable toRun, DTSubsystem... requirements) {
    super(() -> {
    }, toRun, () -> {
    }, () -> {
    }, () -> false, requirements);
  }
}