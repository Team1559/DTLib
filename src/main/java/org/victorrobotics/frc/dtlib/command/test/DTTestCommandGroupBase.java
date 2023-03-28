package org.victorrobotics.frc.dtlib.command.test;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class DTTestCommandGroupBase<IMPLEMENTATION extends CommandBase> extends CommandBase
        implements DTTestCommand {
    private final IMPLEMENTATION    commandGroup;
    protected final DTTestCommand[] commands;

    protected DTTestCommandGroupBase(Function<Command[], IMPLEMENTATION> constructor,
            DTTestCommand... commands) {
        this.commands = commands.clone();
        commandGroup = constructor.apply(commands);
        CommandScheduler.getInstance()
                        .registerComposedCommands(commandGroup);
        addRequirements(commandGroup.getRequirements()
                                    .toArray(Subsystem[]::new));
    }

    @Override
    public final void initialize() {
        commandGroup.initialize();
    }

    @Override
    public final void execute() {
        commandGroup.execute();
    }

    @Override
    public void end(boolean interrupted) {
        commandGroup.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return commandGroup.isFinished();
    }

    @Override
    public boolean wasSuccessful() {
        for (DTTestCommand command : commands) {
            if (!command.wasSuccessful()) {
                return false;
            }
        }
        return true;
    }
}
