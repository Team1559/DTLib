package org.victorrobotics.frc.dtlib.command.group;

import org.victorrobotics.frc.dtlib.command.DTCommand;
import org.victorrobotics.frc.dtlib.command.DTCommandBase;
import org.victorrobotics.frc.dtlib.command.DTCommandScheduler;
import org.victorrobotics.frc.dtlib.exception.DTIllegalArgumentException;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

public class DTParallelDeadlineGroup extends DTCommandBase {
    private final Map<DTCommand, Boolean> commands;
    private final DTCommand               deadline;

    private boolean runsWhenDisabled;
    private boolean isFinished;
    private boolean isInterruptible;

    public DTParallelDeadlineGroup(DTCommand deadline, DTCommand... commands) {
        this.deadline = Objects.requireNonNull(deadline);
        this.commands = new HashMap<>(commands.length + 1);
        runsWhenDisabled = true;

        addCommands(deadline);
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
            if (!Collections.disjoint(command.getRequirements(), requirements)) {
                throw new DTIllegalArgumentException("Parallel commands may not share requirements", command);
            }
            this.commands.put(command, false);
            requirements.addAll(command.getRequirements());
            runsWhenDisabled &= command.runsWhenDisabled();
            isInterruptible |= command.isInterruptible();
        }
    }

    @Override
    public void initialize() {
        for (Map.Entry<DTCommand, Boolean> commandEntry : commands.entrySet()) {
            commandEntry.getKey()
                        .initialize();
            commandEntry.setValue(true);
        }
        isFinished = false;
    }

    @Override
    public void execute() {
        for (Map.Entry<DTCommand, Boolean> commandEntry : commands.entrySet()) {
            if (!commandEntry.getValue()) {
                continue;
            }
            DTCommand command = commandEntry.getKey();
            command.execute();
            if (command.isFinished()) {
                command.end();
                commandEntry.setValue(false);
                if (command == deadline) {
                    isFinished = true;
                }
            }
        }
    }

    @Override
    public void end() {
        for (Map.Entry<DTCommand, Boolean> commandEntry : commands.entrySet()) {
            if (commandEntry.getValue()) {
                commandEntry.getKey()
                            .interrupt();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public boolean runsWhenDisabled() {
        return runsWhenDisabled;
    }

    @Override
    public boolean isInterruptible() {
        return isInterruptible;
    }
}
