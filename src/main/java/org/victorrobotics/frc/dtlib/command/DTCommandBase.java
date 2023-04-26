package org.victorrobotics.frc.dtlib.command;

import org.victorrobotics.frc.dtlib.DTSubsystem;

import java.util.HashSet;
import java.util.Set;

public abstract class DTCommandBase implements DTCommand {
    protected final Set<DTSubsystem> requirements;
    protected String                 name;

    protected DTCommandBase() {
        requirements = new HashSet<>();
        name = getClass().getSimpleName();
    }

    @Override
    public void initialize() {
        // default implementation empty
    }

    @Override
    public void execute() {
        // default implementation empty
    }

    @Override
    public void end() {
        // default implementation empty
    }

    @Override
    public void interrupt() {
        // default implementation empty
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean wasSuccessful() {
        return true;
    }

    @Override
    public final Set<DTSubsystem> getRequirements() {
        return requirements;
    }

    @Override
    public String getName() {
        return name;
    }

    protected final void addRequirements(DTSubsystem... requirements) {
        for (DTSubsystem subsystem : requirements) {
            this.requirements.add(subsystem);
        }
    }
}
