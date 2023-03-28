package org.victorrobotics.frc.dtlib.command.util;

import org.victorrobotics.frc.dtlib.command.test.DTInstantTestCommand;

// Explicitly does nothing when called (cleaner code)
public class DTNullCommand extends DTInstantTestCommand {
    public DTNullCommand() {}

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
