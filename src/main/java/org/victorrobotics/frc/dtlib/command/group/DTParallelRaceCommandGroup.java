package org.victorrobotics.frc.dtlib.command.group;

import org.victorrobotics.frc.dtlib.DTSubsystem;
import org.victorrobotics.frc.dtlib.command.DTCommand;
import org.victorrobotics.frc.dtlib.command.DTCommandBase;
import org.victorrobotics.frc.dtlib.command.DTCommandScheduler;
import org.victorrobotics.frc.dtlib.exception.DTIllegalArgumentException;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

public class DTParallelRaceCommandGroup extends DTCommandBase {
    private final Map<DTCommand, Boolean> raceCommands;

    private boolean runsWhenDisabled;
    private boolean isFinished;
    private boolean isInterruptible;
    private boolean success;

    public DTParallelRaceCommandGroup(DTCommand... commands) {
        raceCommands = new HashMap<>(commands.length);
        isInterruptible = false;
        runsWhenDisabled = true;
        isFinished = true;
        addCommands(commands);
    }

    public void addCommands(DTCommand... commands) {
        if (!isFinished) {
            throw new IllegalStateException("Cannot add commands to a running composition");
        } else if (commands == null || commands.length == 0) {
            return;
        }
        DTCommandScheduler.registerComposedCommands(commands);

        for (DTCommand command : commands) {
            Set<DTSubsystem> commandReqs = command.getRequirements();
            if (!Collections.disjoint(requirements, commandReqs)) {
                throw new DTIllegalArgumentException("Parallel commands may not share requirements", command);
            }
            raceCommands.put(command, false);
            requirements.addAll(commandReqs);
            runsWhenDisabled &= command.runsWhenDisabled();
            isInterruptible |= command.isInterruptible();
        }
    }

    @Override
    public void initialize() {
        isFinished = false;
        success = true;
        for (DTCommand command : raceCommands.keySet()) {
            command.initialize();
        }
    }

    @Override
    public void execute() {
        for (Entry<DTCommand, Boolean> commandEntry : raceCommands.entrySet()) {
            DTCommand command = commandEntry.getKey();
            command.execute();
            if (command.isFinished()) {
                command.end();
                success &= command.wasSuccessful();
                commandEntry.setValue(true);
            }
        }
    }

    @Override
    public void end() {
        for (Entry<DTCommand, Boolean> commandEntry : raceCommands.entrySet()) {
            if (commandEntry.getValue()) {
                continue;
            }
            DTCommand command = commandEntry.getKey();
            command.interrupt();
            success &= command.wasSuccessful();
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public boolean wasSuccessful() {
        return success;
    }

    @Override
    public boolean isInterruptible() {
        return isInterruptible;
    }

    @Override
    public boolean runsWhenDisabled() {
        return runsWhenDisabled;
    }

    @Override
    public DTParallelRaceCommandGroup raceWith(DTCommand... parallel) {
        addCommands(parallel);
        return this;
    }
}
