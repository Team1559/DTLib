package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.DTRobot;
import org.victorrobotics.dtlib.log.DTLog;
import org.victorrobotics.dtlib.log.LogWriter;
import org.victorrobotics.dtlib.log.Watchdog;
import org.victorrobotics.dtlib.subsystem.Subsystem;

import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.WeakHashMap;

/**
 * The scheduler responsible for managing commands and subsystems.
 *
 * @see Command
 * @see Subsystem
 */
public final class CommandScheduler {
  private static final Set<Command> COMPOSED_COMMANDS  =
      Collections.newSetFromMap(new WeakHashMap<>());
  private static final Set<Command> SCHEDULED_COMMANDS = new LinkedHashSet<>();

  private static final Set<Command> COMMANDS_TO_SCHEDULE = new LinkedHashSet<>();
  private static final Set<Command> COMMANDS_TO_CANCEL   = new LinkedHashSet<>();

  private static final Map<Subsystem, Command> REQUIRING_COMMANDS = new LinkedHashMap<>();

  private static final List<Runnable> CALLBACKS = new LinkedList<>();

  private static boolean schedulerDisabled;
  private static boolean isRunning;

  private CommandScheduler() {}

  /**
   * Runs a single iteration of the scheduler. The execution occurs in the
   * following order:
   * <ol>
   * <li>Inputs are polled, and input-bound actions are run</li>
   * <li>Subsystem periodic methods are called.</li>
   * <li>Scheduled commands are executed.</li>
   * <li>End conditions are checked on scheduled commands, and finished commands
   * have their end methods called and are removed.</li>
   * <li>Any subsystems not being used as requirements have their default
   * commands started.</li>
   * </ol>
   */
  public static void run() {
    if (schedulerDisabled) return;

    CALLBACKS.forEach(Runnable::run);

    for (Subsystem subsystem : REQUIRING_COMMANDS.keySet()) {
      try {
        Watchdog.startEpoch();
        subsystem.periodic();
        Watchdog.addEpoch(subsystem.getName() + ".periodic()");
      } catch (RuntimeException e) {
        LogWriter.logException(e, DTLog.Level.WARN);
      }

      if (DTRobot.isSimulation()) {
        try {
          Watchdog.startEpoch();
          subsystem.simulationPeriodic();
          Watchdog.addEpoch(subsystem.getName() + ".simulationPeriodic()");
        } catch (RuntimeException e) {
          LogWriter.logException(e, DTLog.Level.WARN);
        }
      }
    }

    isRunning = true;
    for (Iterator<Command> iterator = SCHEDULED_COMMANDS.iterator(); iterator.hasNext();) {
      Command command = iterator.next();

      if (!DTRobot.getCurrentMode().isEnabled && !command.runsWhenDisabled()) {
        try {
          Watchdog.startEpoch();
          command.interrupt();
          Watchdog.addEpoch(command.getName() + ".interrupt()");
        } catch (RuntimeException e) {
          LogWriter.logException(e, DTLog.Level.WARN);
        }

        command.getRequirements()
               .forEach(s -> REQUIRING_COMMANDS.put(s, null));
        iterator.remove();
        continue;
      }

      boolean exception = false;
      try {
        Watchdog.startEpoch();
        command.execute();
        Watchdog.addEpoch(command.getName() + ".execute()");
      } catch (RuntimeException e) {
        LogWriter.logException(e, DTLog.Level.WARN);
        exception = true;
      }

      boolean finished = true;
      try {
        finished = command.isFinished();
      } catch (RuntimeException e) {
        LogWriter.logException(e, DTLog.Level.WARN);
        exception = true;
      }

      if (exception) {
        try {
          Watchdog.startEpoch();
          command.interrupt();
          Watchdog.addEpoch(command.getName() + ".interrupt()");
        } catch (RuntimeException e) {
          LogWriter.logException(e, DTLog.Level.WARN);
        }

        iterator.remove();
        command.getRequirements()
               .forEach(s -> REQUIRING_COMMANDS.put(s, null));
      } else if (finished) {
        try {
          Watchdog.startEpoch();
          command.end();
          Watchdog.addEpoch(command.getName() + ".end()");
        } catch (RuntimeException e) {
          LogWriter.logException(e, DTLog.Level.WARN);
        }

        iterator.remove();
        command.getRequirements()
               .forEach(s -> REQUIRING_COMMANDS.put(s, null));
      }
    }
    isRunning = false;

    COMMANDS_TO_SCHEDULE.forEach(CommandScheduler::schedule);
    COMMANDS_TO_SCHEDULE.clear();

    COMMANDS_TO_CANCEL.forEach(CommandScheduler::cancel);
    COMMANDS_TO_CANCEL.clear();

    for (Entry<Subsystem, Command> entry : REQUIRING_COMMANDS.entrySet()) {
      if (entry.getValue() == null) {
        schedule(entry.getKey()
                      .getDefaultCommand());
      }
    }
  }

  /**
   * Whether the given commands are running. Note that this only works on
   * commands that are directly scheduled by the scheduler; it will not work on
   * commands inside compositions, as the scheduler does not see them.
   *
   * @param command the command to query
   * @return whether the command is currently scheduled
   */
  public static boolean isScheduled(Command command) {
    return SCHEDULED_COMMANDS.contains(command);
  }

  /**
   * Schedules multiple commands for execution.
   *
   * @param commands the commands to schedule. No-op on null.
   * @see #schedule(Command)
   */
  public static void schedule(Command... commands) {
    for (Command command : commands) {
      schedule(command);
    }
  }

  /**
   * Schedules a {@link Command} for execution, provided all of the following
   * conditions are met:
   * <ul>
   * <li>The command is not {@code null}</li>
   * <li>The command is not already scheduled</li>
   * <li>The command does not belong to a composed command group</li>
   * <li>The robot is enabled, or the command runs when disabled</li>
   * <li>The command's required subsystems aren't currently busy, or their
   * commands can all be canceled</li>
   * </ul>
   *
   * @param command the command to schedule.
   * @see CommandScheduler#isScheduled(DTCommand)
   * @see Command#isInterruptible()
   * @see Command#runsWhenDisabled()
   * @see DTRobot#getCurrentMode()
   */
  public static boolean schedule(Command command) {
    if (command == null) {
      LogWriter.warn("Tried to schedule a null command");
      return false;
    } else if (COMPOSED_COMMANDS.contains(command)) {
      LogWriter.warn("Tried to schedule a composed command");
      return false;
    }

    if (schedulerDisabled || isScheduled(command)
        || (!DTRobot.getCurrentMode().isEnabled && !command.runsWhenDisabled())) {
      return false;
    }

    if (isRunning) {
      COMMANDS_TO_SCHEDULE.add(command);
      COMMANDS_TO_CANCEL.remove(command);
      return true;
    }

    Set<Subsystem> requirements = command.getRequirements();

    Map<Subsystem, Command> map = new LinkedHashMap<>(requirements.size());
    for (Subsystem requirement : requirements) {
      Command requiring = getRequiringCommand(requirement);
      if (requiring == null) continue;
      if (!requiring.isInterruptible()) return false;
      map.put(requirement, requiring);
    }

    for (Map.Entry<Subsystem, Command> entry : map.entrySet()) {
      cancel(entry.getValue());
      REQUIRING_COMMANDS.put(entry.getKey(), command);
    }

    try {
      Watchdog.startEpoch();
      command.initialize();
      Watchdog.addEpoch(command.getName() + ".initialize()");
    } catch (RuntimeException e) {
      LogWriter.logException(e, DTLog.Level.WARN);
      return false;
    }

    SCHEDULED_COMMANDS.add(command);
    return true;
  }

  /**
   * Cancels commands.
   *
   * @param commands the commands to cancel
   * @see #cancel(Command)
   */
  public static void cancel(Command... commands) {
    for (Command command : commands) {
      cancel(command);
    }
  }

  /**
   * Cancels the given command. The scheduler will call the command's
   * interrupt() method, indicating it was canceled (as opposed to finishing
   * normally).
   * <p>
   * Commands will be canceled regardless of interruption behavior.
   *
   * @param command the command to cancel
   * @see Command#interrupt()
   */
  public static void cancel(Command command) {
    if (command == null) {
      LogWriter.warn("Tried to cancel a null command");
      return;
    } else if (!isScheduled(command)) return;

    if (isRunning) {
      COMMANDS_TO_CANCEL.add(command);
      COMMANDS_TO_SCHEDULE.remove(command);
      return;
    }

    SCHEDULED_COMMANDS.remove(command);
    command.getRequirements()
           .forEach(s -> REQUIRING_COMMANDS.put(s, null));

    try {
      Watchdog.startEpoch();
      command.interrupt();
      Watchdog.addEpoch(command.getName() + ".interrupt()");
    } catch (RuntimeException e) {
      LogWriter.logException(e, DTLog.Level.WARN);
    }
  }

  /** Cancels all commands that are currently scheduled. */
  public static void cancelAll() {
    if (isRunning) {
      COMMANDS_TO_CANCEL.addAll(SCHEDULED_COMMANDS);
      return;
    }

    SCHEDULED_COMMANDS.forEach(command -> {
      try {
        Watchdog.startEpoch();
        command.interrupt();
        Watchdog.addEpoch(command.getName() + ".interrupt()");
      } catch (RuntimeException e) {
        LogWriter.logException(e, DTLog.Level.WARN);
      }
      command.getRequirements()
             .forEach(s -> REQUIRING_COMMANDS.put(s, null));
    });
    SCHEDULED_COMMANDS.clear();
  }

  /**
   * Registers a subsystem with the scheduler. This is called automatically by
   * the {@link Subsystem} constructor.
   *
   * @param subsystem the subsystem to register
   * @see Subsystem#Subsystem() Subsystem()
   */
  public static void registerSubsystem(Subsystem subsystem) {
    if (subsystem == null) {
      LogWriter.warn("Tried to register a null subsystem");
      return;
    }

    REQUIRING_COMMANDS.putIfAbsent(subsystem, null);
  }

  /**
   * Returns the command currently requiring a given subsystem. Null if no
   * command is currently requiring the subsystem
   *
   * @param subsystem the subsystem to be inquired about
   * @return the command currently requiring the subsystem, or null if no
   *         command is currently scheduled
   */
  public static Command getRequiringCommand(Subsystem subsystem) {
    return REQUIRING_COMMANDS.get(subsystem);
  }

  /** Disables the command scheduler. */
  public static void disable() {
    schedulerDisabled = true;
  }

  /** Enables the command scheduler. */
  public static void enable() {
    schedulerDisabled = false;
  }

  /**
   * Register a command as composed. An exception will be thrown if these
   * commands are scheduled directly or added to another composition.
   *
   * @param command the command to register
   * @throws IllegalArgumentException if the given command has already been
   *         composed
   */
  public static void registerComposed(Command command) {
    requireNotComposed(command);
    COMPOSED_COMMANDS.add(command);
  }

  /**
   * Register commands as composed. An exception will be thrown if these
   * commands are scheduled directly or added to a composition.
   *
   * @param commands the commands to register
   * @throws IllegalArgumentException if the given commands have already been
   *         composed
   */
  public static void registerComposed(Command... commands) {
    registerComposed(Set.of(commands));
  }

  /**
   * Register commands as composed. An exception will be thrown if these
   * commands are scheduled directly or added to a composition.
   *
   * @param commands the commands to register
   * @throws IllegalArgumentException if the given commands have already been
   *         composed
   */
  public static void registerComposed(Collection<Command> commands) {
    requireNotComposed(commands);
    COMPOSED_COMMANDS.addAll(commands);

  }

  private static void requireNotComposed(Command command) {
    if (COMPOSED_COMMANDS.contains(command)) {
      throw new IllegalArgumentException("composed commands may not be scheduled or added to another composition");
    }
  }

  private static void requireNotComposed(Collection<Command> commands) {
    if (!Collections.disjoint(commands, COMPOSED_COMMANDS)) {
      throw new IllegalArgumentException("composed commands may not be scheduled or added to another composition");
    }
  }

  /**
   * Adds a callback to run automatically every cycle.
   *
   * @param r the task to run
   */
  public static void bindCallback(Runnable r) {
    CALLBACKS.add(r);
  }
}
