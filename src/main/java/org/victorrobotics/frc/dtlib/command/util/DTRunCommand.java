package org.victorrobotics.frc.dtlib.command.util;

import org.victorrobotics.frc.dtlib.DTSubsystem;

public class DTRunCommand extends DTFunctionalCommand {
    public DTRunCommand(Runnable toRun, DTSubsystem... requirements) {
        super(() -> {}, toRun, () -> {}, () -> {}, () -> false, requirements);
    }
}
