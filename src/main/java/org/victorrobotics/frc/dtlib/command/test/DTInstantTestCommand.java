package org.victorrobotics.frc.dtlib.command.test;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DTInstantTestCommand extends InstantCommand implements DTTestCommand {
    public DTInstantTestCommand() {}

    public DTInstantTestCommand(Runnable toRun, Subsystem... requirements) {
        super(toRun, requirements);
    }
}
