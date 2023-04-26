package org.victorrobotics.frc.dtlib.command.util;

import org.victorrobotics.frc.dtlib.DTSubsystem;
import org.victorrobotics.frc.dtlib.command.DTCommandBase;

import java.util.function.BooleanSupplier;

public class DTFunctionalCommand extends DTCommandBase {
    protected Runnable        init;
    protected Runnable        execute;
    protected Runnable        onEnd;
    protected Runnable        onInterrupt;
    protected BooleanSupplier isFinished;

    public DTFunctionalCommand(DTSubsystem... requirements) {
        addRequirements(requirements);
    }

    public DTFunctionalCommand(Runnable init, Runnable execute, Runnable onEnd, Runnable onInterrupt,
            BooleanSupplier isFinished, DTSubsystem... requirements) {
        this(requirements);
        this.init = init;
        this.execute = execute;
        this.onEnd = onEnd;
        this.onInterrupt = onInterrupt;
        this.isFinished = isFinished;
    }

    @Override
    public void initialize() {
        if (init != null) {
            init.run();
        }
    }

    @Override
    public void execute() {
        if (execute != null) {
            execute.run();
        }
    }

    @Override
    public void end() {
        if (onEnd != null) {
            onEnd.run();
        }
    }

    @Override
    public void interrupt() {
        if (onInterrupt != null) {
            onInterrupt.run();
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished == null || isFinished.getAsBoolean();
    }
}
