package org.victorrobotics.frc.dtlib.command.group;

import org.victorrobotics.frc.dtlib.command.DTCommand;
import org.victorrobotics.frc.dtlib.command.DTCommandBase;
import org.victorrobotics.frc.dtlib.command.DTCommandScheduler;

import java.util.ArrayList;
import java.util.List;

public class DTSequentialCommandGroup extends DTCommandBase {
    private final List<DTCommand> sequentialCommands;
    private int                   cmdIndex;
    private boolean               runsWhenDisabled;
    private boolean               isInterruptible;
    private boolean               success;

    public DTSequentialCommandGroup(DTCommand... commands) {
        sequentialCommands = new ArrayList<>();
        cmdIndex = -1;
        addCommands(commands);
    }

    public final void addCommands(DTCommand... commands) {
        addCommands(sequentialCommands.size(), commands);
    }

    public final void addCommands(int index, DTCommand... commands) {
        if (cmdIndex != -1) {
            throw new IllegalStateException("Cannot add commands to a running composition");
        }
        DTCommandScheduler.registerComposedCommands(commands);

        sequentialCommands.addAll(index, List.of(commands));
        for (DTCommand command : commands) {
            requirements.addAll(command.getRequirements());
            runsWhenDisabled &= command.runsWhenDisabled();
            isInterruptible |= command.isInterruptible();
        }
    }

    @Override
    public final void initialize() {
        cmdIndex = 0;
        if (!sequentialCommands.isEmpty()) {
            sequentialCommands.get(0)
                              .initialize();
        }
    }

    @Override
    public final void execute() {
        if (sequentialCommands.isEmpty()) {
            return;
        }

        DTCommand current = sequentialCommands.get(cmdIndex);
        current.execute();

        // Multiple commands can execute per loop cycle if they all finish
        while (current.isFinished()) {
            current.end();
            success &= current.wasSuccessful();
            cmdIndex++;
            if (cmdIndex == sequentialCommands.size()) {
                return;
            }
            current = sequentialCommands.get(cmdIndex);
            current.initialize();
            current.execute();
        }
    }

    @Override
    public final void end() {
        cmdIndex = -1;
    }

    @Override
    public final void interrupt() {
        if (!sequentialCommands.isEmpty() && cmdIndex > -1 && cmdIndex < sequentialCommands.size()) {
            DTCommand command = sequentialCommands.get(cmdIndex);
            command.interrupt();
            success &= command.wasSuccessful();
        }
        success &= cmdIndex >= sequentialCommands.size() - 1;
        cmdIndex = -1;
    }

    @Override
    public boolean isFinished() {
        return cmdIndex == sequentialCommands.size();
    }

    @Override
    public boolean runsWhenDisabled() {
        return runsWhenDisabled;
    }

    @Override
    public boolean isInterruptible() {
        return isInterruptible;
    }

    @Override
    public boolean wasSuccessful() {
        return success;
    }
}
