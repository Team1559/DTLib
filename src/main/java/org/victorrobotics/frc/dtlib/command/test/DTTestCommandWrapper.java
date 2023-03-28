package org.victorrobotics.frc.dtlib.command.test;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DTTestCommandWrapper<T extends Command> extends CommandBase implements DTTestCommand {
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
    public void initialize() {
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public boolean wasSuccessful() {
        return wasSuccessful == null || wasSuccessful.getAsBoolean();
    }
}
