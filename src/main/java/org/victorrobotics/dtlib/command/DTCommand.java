package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.DTSubsystem;
import org.victorrobotics.dtlib.command.group.DTConditionalCommand;
import org.victorrobotics.dtlib.command.group.DTParallelCommandGroup;
import org.victorrobotics.dtlib.command.group.DTParallelDeadlineGroup;
import org.victorrobotics.dtlib.command.group.DTParallelRaceCommandGroup;
import org.victorrobotics.dtlib.command.group.DTSequentialCommandGroup;
import org.victorrobotics.dtlib.command.util.DTNullCommand;
import org.victorrobotics.dtlib.command.util.DTProxyCommand;
import org.victorrobotics.dtlib.command.util.DTRecoveryCommand;
import org.victorrobotics.dtlib.command.util.DTRepeatCommand;
import org.victorrobotics.dtlib.command.util.DTTargetCommand;
import org.victorrobotics.dtlib.command.util.DTWaitCommand;
import org.victorrobotics.dtlib.command.util.DTWaitUntilCommand;
import org.victorrobotics.dtlib.command.util.DTWrapperCommand;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public interface DTCommand {
  Set<DTSubsystem> getRequirements();

  void initialize();

  void execute();

  boolean isFinished();

  void end();

  default void interrupt() {
    end();
  }

  default boolean wasSuccessful() {
    return true;
  }

  default boolean isInterruptible() {
    return true;
  }

  default boolean runsWhenDisabled() {
    return false;
  }

  default String getName() {
    return getClass().getSimpleName();
  }

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

  default DTTargetCommand withName(String name) {
    return new DTTargetCommand(this) {
      @Override
      public String getName() {
        return name;
      }
    };
  }

  default DTParallelRaceCommandGroup withTimeout(double seconds) {
    return raceWith(new DTWaitCommand(seconds));
  }

  default DTParallelRaceCommandGroup until(BooleanSupplier condition) {
    return raceWith(new DTWaitUntilCommand(condition));
  }

  default DTConditionalCommand unless(BooleanSupplier condition) {
    return new DTConditionalCommand(new DTNullCommand(), this, condition);
  }

  default DTConditionalCommand onlyIf(BooleanSupplier condition) {
    return new DTConditionalCommand(this, new DTNullCommand(), condition);
  }

  default DTSequentialCommandGroup beforeStarting(DTCommand before) {
    return new DTSequentialCommandGroup(this).beforeStarting(before);
  }

  default DTSequentialCommandGroup andThen(DTCommand... next) {
    return new DTSequentialCommandGroup(this).andThen(next);
  }

  default DTParallelCommandGroup alongWith(DTCommand... parallel) {
    return new DTParallelCommandGroup(this).alongWith(parallel);
  }

  default DTParallelRaceCommandGroup raceWith(DTCommand... parallel) {
    return new DTParallelRaceCommandGroup(this).raceWith(parallel);
  }

  default DTRepeatCommand repeatedly() {
    return new DTRepeatCommand(this);
  }

  default DTParallelDeadlineGroup repeatUntil(BooleanSupplier condition) {
    return new DTParallelDeadlineGroup(new DTWaitUntilCommand(condition),
        new DTRepeatCommand(this));
  }

  default DTRecoveryCommand catchExceptions() {
    return new DTRecoveryCommand(this);
  }

  default DTProxyCommand proxy() {
    return new DTProxyCommand(this);
  }

  default DTTargetCommand overrideDisable(boolean runsWhenDisabled) {
    return new DTTargetCommand(this) {
      @Override
      public boolean runsWhenDisabled() {
        return runsWhenDisabled;
      }
    };
  }

  default DTTargetCommand overrideInterrupt(boolean isInterruptible) {
    return new DTTargetCommand(this) {
      @Override
      public boolean isInterruptible() {
        return isInterruptible;
      }
    };
  }

  static DTWrapperCommand wrap(Command command) {
    return new DTWrapperCommand(command);
  }
}
