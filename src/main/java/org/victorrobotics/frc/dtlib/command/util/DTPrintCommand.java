package org.victorrobotics.frc.dtlib.command.util;

import org.victorrobotics.frc.dtlib.command.test.DTInstantTestCommand;

public class DTPrintCommand extends DTInstantTestCommand {
    public DTPrintCommand(String message) {
        super(() -> System.out.println(message));
    }
}
