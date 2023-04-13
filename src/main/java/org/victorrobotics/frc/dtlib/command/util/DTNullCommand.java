package org.victorrobotics.frc.dtlib.command.util;

import org.victorrobotics.frc.dtlib.command.test.DTInstantTestCommand;

// Explicitly does nothing when called (cleaner code)
public class DTNullCommand extends DTInstantTestCommand {
    private static final Runnable NULL_RUNNABLE = () -> {
    };

    public DTNullCommand() {
        super(NULL_RUNNABLE);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
