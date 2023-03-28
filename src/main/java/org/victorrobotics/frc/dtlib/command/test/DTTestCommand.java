package org.victorrobotics.frc.dtlib.command.test;

import edu.wpi.first.wpilibj2.command.Command;

public interface DTTestCommand extends Command {
    default boolean wasSuccessful() {
        return true;
    }

    default DTSequentialTestCommandGroup andThenTest(DTTestCommand next, DTTestCommand... more) {
        return new DTSequentialTestCommandGroup(makeArray(this, next, more));
    }

    default DTSequentialTestCommandGroup beforeStartingTest(DTTestCommand before) {
        return new DTSequentialTestCommandGroup(makeArray(before, this));
    }

    default DTParallelTestCommandGroup alongWithTest(DTTestCommand other) {
        return new DTParallelTestCommandGroup(makeArray(this, other));
    }

    private static DTTestCommand[] makeArray(DTTestCommand first, DTTestCommand next,
            DTTestCommand... more) {
        if (more == null || more.length == 0) {
            return new DTTestCommand[] { first, next };
        } else {
            DTTestCommand[] commands = new DTTestCommand[more.length + 2];
            commands[0] = first;
            commands[1] = next;
            System.arraycopy(more, 0, commands, 2, more.length);
            return commands;
        }
    }
}
