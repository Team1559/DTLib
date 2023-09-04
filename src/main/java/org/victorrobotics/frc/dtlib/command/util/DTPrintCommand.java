package org.victorrobotics.frc.dtlib.command.util;

public class DTPrintCommand extends DTInstantCommand {
    public DTPrintCommand(String message) {
        super(() -> System.out.println(message));
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
