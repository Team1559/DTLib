package org.victorrobotics.frc.dtlib.command.test;

import org.victorrobotics.frc.dtlib.command.DTCommandBase;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class DTInstantTestCommand extends DTCommandBase implements DTTestCommand {
    private static final Runnable NULL_RUNNABLE = () -> {
    };

    private final Runnable action;

    public DTInstantTestCommand() {
        action = NULL_RUNNABLE;
    }

    public DTInstantTestCommand(Runnable toRun, Subsystem... requirements) {
        action = toRun;
        addRequirements(requirements);
    }

    @Override
    protected void start() {
        action.run();
    }

    @Override
    protected void run() {
        // nothing
    }

    @Override
    protected boolean isComplete() {
        return true;
    }

    @Override
    protected void finish(boolean interrupted) {
        // nothing
    }
}
