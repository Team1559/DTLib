package org.victorrobotics.frc.dtlib.command;

import org.victorrobotics.frc.dtlib.DTSubsystem;
import org.victorrobotics.frc.dtlib.command.group.DTParallelCommandGroup;
import org.victorrobotics.frc.dtlib.command.group.DTSequentialCommandGroup;

import java.util.Set;
import java.util.function.BooleanSupplier;

public interface DTCommand {
    void initialize();

    void execute();

    void end();

    void interrupt();

    boolean isFinished();

    boolean wasSuccessful();

    Set<DTSubsystem> getRequirements();

    default boolean hasRequirement(DTSubsystem requirement) {
        return getRequirements().contains(requirement);
    }

    default void schedule() {
        DTCommandScheduler.schedule(this);
    }

    default void cancel() {
        DTCommandScheduler.cancel(this);
    }

    default boolean isScheduled() {
        return DTCommandScheduler.isScheduled(this);
    }

    default String getName() {
        return getClass().getSimpleName();
    }

    default boolean isInterruptible() {
        return true;
    }

    default boolean runsWhenDisabled() {
        return false;
    }

    default DTCommand withName(String name) {
        throw new UnsupportedOperationException();
    }

    default DTCommand withTimeout(double seconds) {
        throw new UnsupportedOperationException();
    }

    default DTCommand until(BooleanSupplier condition) {
        throw new UnsupportedOperationException();
    }

    default DTCommand unless(BooleanSupplier condition) {
        throw new UnsupportedOperationException();
    }

    default DTCommand beforeStarting(DTCommand before) {
        throw new UnsupportedOperationException();
    }

    default DTSequentialCommandGroup andThen(DTCommand... next) {
        DTSequentialCommandGroup group = (this instanceof DTSequentialCommandGroup) ? (DTSequentialCommandGroup) this
                : new DTSequentialCommandGroup(this);
        group.addCommands(next);
        return group;
    }

    default DTParallelCommandGroup alongWith(DTCommand... parallel) {
        DTParallelCommandGroup group = (this instanceof DTParallelCommandGroup) ? (DTParallelCommandGroup) this
                : new DTParallelCommandGroup(this);
        group.addCommands(parallel);
        return group;
    }

    default DTCommand raceWith(DTCommand parallel) {
        throw new UnsupportedOperationException();
    }

    default DTCommand repeatedly() {
        throw new UnsupportedOperationException();
    }

    default DTCommand catchExceptions() {
        throw new UnsupportedOperationException();
    }
}
