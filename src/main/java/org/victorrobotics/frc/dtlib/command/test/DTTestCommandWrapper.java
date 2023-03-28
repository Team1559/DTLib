package org.victorrobotics.frc.dtlib.command.test;

import org.victorrobotics.frc.dtlib.command.DTCommandBase;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DTTestCommandWrapper<T extends Command> extends DTCommandBase implements DTTestCommand {
    private final T               command;
    private final BooleanSupplier wasSuccessful;

    public DTTestCommandWrapper(T command) {
        this(command, null);
    }

    public DTTestCommandWrapper(T command, BooleanSupplier wasSuccessful) {
        this.command = command;
        this.wasSuccessful = wasSuccessful;
        CommandScheduler.getInstance()
                        .registerComposedCommands(command);
        addRequirements(command.getRequirements()
                               .toArray(Subsystem[]::new));
    }

    @Override
    public void start() {
        command.initialize();
    }

    @Override
    public void run() {
        command.execute();
    }

    @Override
    public void finish(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public boolean isComplete() {
        return command.isFinished();
    }

    @Override
    public boolean wasSuccessful() {
        return wasSuccessful == null || wasSuccessful.getAsBoolean();
    }
}
