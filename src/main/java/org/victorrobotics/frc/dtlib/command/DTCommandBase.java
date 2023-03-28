package org.victorrobotics.frc.dtlib.command;

import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class DTCommandBase extends CommandBase {
    protected abstract void start();

    protected abstract void run();

    protected abstract boolean isComplete();

    protected abstract void finish(boolean interrupted);

    @Override
    public final void initialize() {
        try {
            start();
        } catch (Exception e) {
            cancel();
        }
    }

    @Override
    public final void execute() {
        try {
            run();
        } catch (Exception e) {
            cancel();
        }
    }

    @Override
    public final boolean isFinished() {
        try {
            return isComplete();
        } catch (Exception e) {
            cancel();
            return false;
        }
    }

    @Override
    public final void end(boolean interrupted) {
        try {
            finish(interrupted);
        } catch (Exception e) {
            // Nothing to do, command is already finished
        }
    }
}
