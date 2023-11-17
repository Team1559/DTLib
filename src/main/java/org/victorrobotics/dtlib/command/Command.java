package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.DTRobot;
import org.victorrobotics.dtlib.subsystem.Subsystem;

import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

/**
 * A state machine representing an action to be performed by the robot. Commands
 * are run by the {@link CommandScheduler}, and can be composed into groups to
 * allow users to build multistep actions without the need to write complex
 * state-handling logic themselves.
 */
public abstract class Command {
  private final Set<Subsystem> requirements;
  private final Set<Subsystem> unmodifiableReqs;

  /**
   * Constructs a new CommandBase
   */
  protected Command() {
    requirements = new LinkedHashSet<>();
    unmodifiableReqs = Collections.unmodifiableSet(requirements);
  }

  /**
   * The initial subroutine of the command. Called once each time a command is
   * scheduled.
   */
  public abstract void initialize();

  /**
   * The main body of the command. Called repeatedly while a command is
   * scheduled.
   */
  public abstract void execute();

  /**
   * The action to take when the command ends normally. Called once
   * {@link #isFinished()} returns true.
   * <p>
   * Do not schedule commands here. Use {@link #andThen(Command...)} instead.
   */
  public abstract void end();

  /**
   * Whether the command has finished. Once a command finishes, the scheduler
   * will call its {@link #end()} method and unschedule it.
   * <p>
   * Unless overriden, commands will never finish.
   *
   * @return whether the command has finished
   */
  public abstract boolean isFinished();

  /**
   * The action to take when the scheduler interrupts the command (by canceling
   * it, throwing an exception, or scheduling a conflicting command).
   * <p>
   * Unless overriden, this delegates to the command's {@link #end()} method.
   *
   * @see #end
   * @see CommandScheduler#cancel(Command)
   */
  public void interrupt() {
    end();
  }

  /**
   * Whether the command completed successfully. Used to determine pass/fail of
   * a robot self test.
   * <p>
   * Unless overriden, commands are always successful.
   *
   * @return whether the command completed without issue
   * @see DTRobot#getSelfTestCommand()
   */
  public boolean wasSuccessful() {
    return true;
  }

  /**
   * Whether the command is permitted to be interrupted by the scheduler. If
   * true (the default), scheduling another command that requires one or more of
   * the command's requirements will result in interruption.
   * <p>
   * Regardless of the result of this method, commands may still be canceled
   * explicitly.
   *
   * @return whether the command can be interrupted
   * @see #interrupt()
   * @see CommandScheduler#cancel(Command)
   */
  public boolean isInterruptible() {
    return true;
  }

  /**
   * Whether the command is permitted to remain scheduled while the robot is
   * disabled. Certain command types, for example {@link PrintCommand}, should
   * not be hindered based on enable status. If false (default), commands will
   * be canceled when the robot becomes disabled, and must be rescheduled to run
   * again.
   *
   * @return whether the command runs when the robot is disabled
   * @see DTRobot#getCurrentMode()
   */
  public boolean runsWhenDisabled() {
    return false;
  }

  /**
   * The name of the command. Defaults to the command class name.
   *
   * @return the name of the command
   */
  public String getName() {
    return getClass().getSimpleName();
  }

  /**
   * The set of subsystems used by this command. Two commands may not use the
   * same subsystem simultaneously. If another command is scheduled that shares
   * a requirement, {@link #isInterruptible()} will be checked and followed. If
   * no subsystems are required, return an empty set (using {@link Set#of()}).
   *
   * @return the set of required subsystems
   */
  public final Set<Subsystem> getRequirements() {
    return unmodifiableReqs;
  }

  /**
   * Adds subsystems to the requirements of the command. The
   * {@link CommandScheduler} will prevent two commands that require the same
   * subsystem from being scheduled simultaneously.
   * <p>
   * Note that the scheduler determines the requirements of a command when it is
   * scheduled, so this method should normally be called from the subclass
   * constructor.
   *
   * @param requirements the subsystems to add
   * @see Command#getRequirements()
   */
  public final void addRequirements(Subsystem requirement) {
    this.requirements.add(requirement);
  }

  /**
   * Adds subsystems to the requirements of the command. The
   * {@link CommandScheduler} will prevent two commands that require the same
   * subsystem from being scheduled simultaneously.
   * <p>
   * Note that the scheduler determines the requirements of a command when it is
   * scheduled, so this method should normally be called from the subclass
   * constructor.
   *
   * @param requirements the subsystems to add
   * @see Command#getRequirements()
   */
  public final void addRequirements(Subsystem... requirements) {
    for (Subsystem subsystem : requirements) {
      addRequirements(subsystem);
    }
  }

  /**
   * Mark subsystems as requirements for this command. Commands are given
   * exclusive access to their requirements while they are scheduled: if another
   * command tries to schedule at the same time, one of them will be canceled.
   *
   * @param requirements the set of subsystems to add
   * @see Command#getRequirements()
   */
  public final void addRequirements(Collection<Subsystem> requirements) {
    this.requirements.addAll(requirements);
  }

  /**
   * @param requirement the subsystem to check
   * @return whether the command requires the subsystem
   */
  public boolean hasRequirement(Subsystem requirement) {
    return getRequirements().contains(requirement);
  }

  /**
   * Schedules the command for execution. This is a convenience method
   * equivalent to
   *
   * <pre>
   * CommandScheduler.schedule(this)
   * </pre>
   *
   * @see CommandScheduler#schedule(Command)
   */
  public void schedule() {
    CommandScheduler.schedule(this);
  }

  /**
   * Cancels the command if currently scheduled. This is a convenience method
   * equivalent to
   *
   * <pre>
   * CommandScheduler.cancel(this)
   * </pre>
   *
   * @see CommandScheduler#cancel(Command)
   */
  public void cancel() {
    CommandScheduler.cancel(this);
  }

  /**
   * Whether the command is currently scheduled for execution. This is a
   * convenience method equivalent to
   *
   * <pre>
   * CommandScheduler.isScheduled(this)
   * </pre>
   *
   * @return whether the command is scheduled
   */
  public boolean isScheduled() {
    return CommandScheduler.isScheduled(this);
  }

  /**
   * Decorates this command with an interrupt timeout. If the timeout is past
   * before this command finishes, it will be interrupted.
   *
   * @param seconds the timeout duration
   * @return the decorated command
   * @see RaceCommandGroup
   */
  public RaceCommandGroup withTimeout(double seconds) {
    return raceWith(new WaitCommand(seconds));
  }

  /**
   * Decorates this command with an interrupt condition. If the condition is met
   * before this command finishes, it will be interrupted.
   *
   * @param condition the interrupt condition
   * @return the decorated command
   * @see RaceCommandGroup
   */
  public RaceCommandGroup until(BooleanSupplier condition) {
    return raceWith(new WaitUntilCommand(condition));
  }

  /**
   * If the condition is true when this command is scheduled, ignores this
   * command's execution.
   *
   * @param condition the condition
   * @return the decorated command
   * @see ConditionalCommand
   */
  public ConditionalCommand unless(BooleanSupplier condition) {
    return new ConditionalCommand(new NullCommand(), this, condition);
  }

  /**
   * If the condition is false when this command is scheduled, ignores this
   * command's execution.
   *
   * @param condition the condition
   * @return the decorated command
   * @see ConditionalCommand
   */
  public ConditionalCommand onlyIf(BooleanSupplier condition) {
    return new ConditionalCommand(this, new NullCommand(), condition);
  }

  /**
   * Decorates this command with commands to run before this command starts.
   *
   * @param before the command(s) to run before this one
   * @return the decorated command
   * @see SequentialCommandGroup
   */
  public SequentialCommandGroup beforeStarting(Command... before) {
    return new SequentialCommandGroup(before).andThen(this);
  }

  /**
   * Decorates this command with commands to be run afterwards in sequence.
   *
   * @param next the command(s) to run next
   * @return the decorated command
   * @see SequentialCommandGroup
   */
  public SequentialCommandGroup andThen(Command... next) {
    return new SequentialCommandGroup(this).andThen(next);
  }

  /**
   * Decorates this command with commands to run parallel to it, ending all
   * commands have ended.
   *
   * @param parallel the command(s) to run in parallel
   * @return the decorated command
   * @see ParallelCommandGroup
   */
  public ParallelCommandGroup alongWith(Command... parallel) {
    return new ParallelCommandGroup(this).alongWith(parallel);
  }

  /**
   * Decorates this command with commands to run parallel to it, ending when the
   * first command ends and interrupting the rest.
   *
   * @param parallel the command(s) to run in parallel
   * @return the decorated command
   * @see RaceCommandGroup
   */
  public RaceCommandGroup raceWith(Command... parallel) {
    return new RaceCommandGroup(this).raceWith(parallel);
  }

  /**
   * Decorates this command to run repeatedly, restarting once it finishes,
   * forever until it is interrupted. It can still be canceled.
   *
   * @return the decorated command
   * @see RepeatCommand
   */
  public RepeatCommand repeatedly() {
    return new RepeatCommand(this);
  }

  /**
   * Decorates this command to run repeatedly, restarting once it finishes,
   * until the specified condition is met or it is interrupted. It can still be
   * canceled.
   *
   * @param condition the end condition
   * @return the decorated command
   * @see RepeatCommand
   * @see RaceCommandGroup
   */
  public RaceCommandGroup repeatUntil(BooleanSupplier condition) {
    return repeatedly().until(condition);
  }

  /**
   * Decorates this command to catch all exceptions, rather than propogating
   * them to the scheduler.
   *
   * @return the decorated command
   * @see RecoveryCommand
   */
  public RecoveryCommand catchExceptions() {
    return new RecoveryCommand(this);
  }

  /**
   * Decorates this command as a proxy target, effectively separating the
   * command's attributes (e.g. name, requirements) from its execution.
   *
   * @return the decorated command
   * @see ProxyCommand
   */
  public ProxyCommand proxy() {
    return new ProxyCommand(this);
  }

  /**
   * Decorates this command to have the given disabled behavior.
   *
   * @param runsWhenDisabled whether to run when the robot is disabled
   * @return the decorated command
   * @see #runsWhenDisabled()
   */
  public TargetCommand overrideDisable(boolean runsWhenDisabled) {
    return new TargetCommand(this) {
      @Override
      public boolean runsWhenDisabled() {
        return runsWhenDisabled;
      }
    };
  }

  /**
   * Decorates this command to have the given interruption behavior.
   *
   * @param isInterruptible whether the command should be interruptible
   * @return the decorated command
   * @see #isInterruptible()
   */
  public TargetCommand overrideInterrupt(boolean isInterruptible) {
    return new TargetCommand(this) {
      @Override
      public boolean isInterruptible() {
        return isInterruptible;
      }
    };
  }

  /**
   * Decorates this command with a new name, which is useful for logging and
   * debugging.
   *
   * @param name the new name
   * @return the decorated command
   * @see #getName()
   */
  public TargetCommand withName(String name) {
    return new TargetCommand(this) {
      @Override
      public String getName() {
        return name;
      }
    };
  }
}
