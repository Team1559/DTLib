package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.subsystem.DTSubsystem;

public class DTRunCommand extends DTFunctionalCommand {
  public DTRunCommand(Runnable toRun, DTSubsystem... requirements) {
    super(() -> {}, toRun, () -> {}, () -> {}, () -> false, requirements);
  }
}
