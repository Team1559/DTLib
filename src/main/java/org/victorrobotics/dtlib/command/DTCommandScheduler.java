package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.DTRobot;
import org.victorrobotics.dtlib.exception.DTIllegalArgumentException;
import org.victorrobotics.dtlib.log.DTLog;
import org.victorrobotics.dtlib.log.DTLogWriter;
import org.victorrobotics.dtlib.log.DTWatchdog;
import org.victorrobotics.dtlib.subsystem.DTSubsystem;

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
 * The scheduler responsible for managing DTCommands and DTSubsystems
 *
 * @see DTCommand
 * @see DTSubsystem
 */
public final class DTCommandScheduler {
  private static final Set<DTCommand> COMPOSED_COMMANDS  =
      Collections.newSetFromMap(new WeakHashMap<>());
  private static final Set<DTCommand> SCHEDULED_COMMANDS = new LinkedHashSet<>();

  private static final Set<DTCommand> COMMANDS_TO_SCHEDULE = new LinkedHashSet<>();
  private static final Set<DTCommand> COMMANDS_TO_CANCEL   = new LinkedHashSet<>();

  private static final Map<DTSubsystem, DTCommand> REQUIRING_COMMANDS = new LinkedHashMap<>();

  private static final List<Runnable> CALLBACKS = new LinkedList<>();

  private static boolean schedulerDisabled;
  private static boolean isRunning;

  private DTCommandScheduler() {}

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

    for (DTSubsystem subsystem : REQUIRING_COMMANDS.keySet()) {
      DTWatchdog.startEpoch();
      subsystem.periodic();
      DTWatchdog.addEpoch(subsystem.getName() + ".periodic()");
      if (DTRobot.isSimulation()) {
        DTWatchdog.startEpoch();
        subsystem.simulationPeriodic();
        DTWatchdog.addEpoch(subsystem.getName() + ".simulationPeriodic()");
      }
    }

    isRunning = true;
    for (Iterator<DTCommand> iterator = SCHEDULED_COMMANDS.iterator(); iterator.hasNext();) {
      DTCommand command = iterator.next();

      if (!DTRobot.getCurrentMode().isEnabled && !command.runsWhenDisabled()) {
        DTWatchdog.startEpoch();
        try {
          command.interrupt();
        } catch (RuntimeException e) {
          DTLogWriter.logException(e, DTLog.Level.WARN);
        }
        command.getRequirements()
               .forEach(s -> REQUIRING_COMMANDS.put(s, null));
        iterator.remove();
        DTWatchdog.addEpoch(command.getName() + ".interrupt()");
        continue;
      }

      DTWatchdog.startEpoch();
      try {
        command.execute();
      } catch (RuntimeException e) {
        DTLogWriter.logException(e, DTLog.Level.WARN);
      }
      DTWatchdog.addEpoch(command.getName() + ".execute()");

      boolean finished = true;
      try {
        finished = command.isFinished();
      } catch (RuntimeException e) {
        DTLogWriter.logException(e, DTLog.Level.WARN);
      }

      if (finished) {
        DTWatchdog.startEpoch();
        try {
          command.end();
        } catch (RuntimeException e) {
          DTLogWriter.logException(e, DTLog.Level.WARN);
        }
        iterator.remove();
        command.getRequirements()
               .forEach(s -> REQUIRING_COMMANDS.put(s, null));
        DTWatchdog.addEpoch(command.getName() + ".end()");
      }
    }
    isRunning = false;

    COMMANDS_TO_SCHEDULE.forEach(DTCommandScheduler::schedule);
    COMMANDS_TO_SCHEDULE.clear();

    COMMANDS_TO_CANCEL.forEach(DTCommandScheduler::cancel);
    COMMANDS_TO_CANCEL.clear();

    for (Entry<DTSubsystem, DTCommand> entry : REQUIRING_COMMANDS.entrySet()) {
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
   * @param command
   *        the command to query
   *
   * @return whether the command is currently scheduled
   */
  public static boolean isScheduled(DTCommand command) {
    return SCHEDULED_COMMANDS.contains(command);
  }

  /**
   * Schedules multiple commands for execution.
   *
   * @param commands
   *        the commands to schedule. No-op on null.
   *
   * @see #schedule(DTCommand)
   */
  public static void schedule(DTCommand... commands) {
    for (DTCommand command : commands) {
      schedule(command);
    }
  }

  /**
   * Schedules a {@link DTCommand} for execution, provided all of the following
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
   * @param command
   *        the command to schedule.
   *
   * @see DTCommandScheduler#isScheduled(DTCommand)
   * @see DTCommand#isInterruptible()
   * @see DTCommand#runsWhenDisabled()
   * @see DTRobot#getCurrentMode()
   */
  public static boolean schedule(DTCommand command) {
    if (command == null) {
      DTLogWriter.warn("Tried to schedule a null command");
      return false;
    } else if (COMPOSED_COMMANDS.contains(command)) {
      DTLogWriter.warn("Tried to schedule a composed command");
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

    Set<DTSubsystem> requirements = command.getRequirements();

    Map<DTSubsystem, DTCommand> map = new LinkedHashMap<>(requirements.size());
    for (DTSubsystem requirement : requirements) {
      DTCommand requiring = getRequiringCommand(requirement);
      if (requiring == null) continue;
      if (!requiring.isInterruptible()) return false;
      map.put(requirement, requiring);
    }

    for (Map.Entry<DTSubsystem, DTCommand> entry : map.entrySet()) {
      cancel(entry.getValue());
      REQUIRING_COMMANDS.put(entry.getKey(), command);
    }

    try {
      command.initialize();
    } catch (RuntimeException e) {
      DTLogWriter.logException(e, DTLog.Level.WARN);
      return false;
    }

    SCHEDULED_COMMANDS.add(command);
    return true;
  }

  /**
   * Cancels commands.
   *
   * @param commands
   *        the commands to cancel
   *
   * @see #cancel(DTCommand)
   */
  public static void cancel(DTCommand... commands) {
    for (DTCommand command : commands) {
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
   * @param command
   *        the command to cancel
   *
   * @see DTCommand#interrupt()
   */
  public static void cancel(DTCommand command) {
    if (command == null) {
      DTLogWriter.warn("Tried to cancel a null command");
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
      command.interrupt();
    } catch (RuntimeException e) {
      DTLogWriter.logException(e, DTLog.Level.WARN);
    }
  }

  /** Cancels all commands that are currently scheduled. */
  public static void cancelAll() {
    if (isRunning) {
      COMMANDS_TO_CANCEL.addAll(SCHEDULED_COMMANDS);
      return;
    }

    Iterator<DTCommand> itr = SCHEDULED_COMMANDS.iterator();
    while (itr.hasNext()) {
      DTCommand command = itr.next();
      itr.remove();
      command.getRequirements()
             .forEach(s -> REQUIRING_COMMANDS.put(s, null));

      try {
        command.interrupt();
      } catch (RuntimeException e) {
        DTLogWriter.logException(e, DTLog.Level.WARN);
      }
    }
  }

  /**
   * Registers a subsystem with the scheduler. This is called automatically by
   * the {@link DTSubsystem} constructor.
   *
   * @param subsystem
   *        the subsystem to register
   *
   * @see DTSubsystem#DTSubsystem() DTSubsystem()
   */
  public static void registerSubsystem(DTSubsystem subsystem) {
    if (subsystem == null) {
      DTLogWriter.warn("Tried to register a null subsystem");
      return;
    }

    REQUIRING_COMMANDS.putIfAbsent(subsystem, null);
  }

  /**
   * Returns the command currently requiring a given subsystem. Null if no
   * command is currently requiring the subsystem
   *
   * @param subsystem
   *        the subsystem to be inquired about
   *
   * @return the command currently requiring the subsystem, or null if no
   *         command is currently scheduled
   */
  public static DTCommand getRequiringCommand(DTSubsystem subsystem) {
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
   * @param command
   *        the command to register
   *
   * @throws DTIllegalArgumentException
   *         if the given command has already been composed
   */
  public static void registerComposed(DTCommand command) {
    requireNotComposed(command);
    COMPOSED_COMMANDS.add(command);
  }

  /**
   * Register commands as composed. An exception will be thrown if these
   * commands are scheduled directly or added to a composition.
   *
   * @param commands
   *        the commands to register
   *
   * @throws DTIllegalArgumentException
   *         if the given commands have already been composed
   */
  public static void registerComposed(DTCommand... commands) {
    registerComposed(Set.of(commands));
  }

  /**
   * Register commands as composed. An exception will be thrown if these
   * commands are scheduled directly or added to a composition.
   *
   * @param commands
   *        the commands to register
   *
   * @throws DTIllegalArgumentException
   *         if the given commands have already been composed
   */
  public static void registerComposed(Collection<DTCommand> commands) {
    requireNotComposed(commands);
    COMPOSED_COMMANDS.addAll(commands);

  }

  private static void requireNotComposed(DTCommand command) {
    if (COMPOSED_COMMANDS.contains(command)) {
      throw new DTIllegalArgumentException(command,
                                           "composed commands may not be scheduled or added to another composition");
    }
  }

  private static void requireNotComposed(Collection<DTCommand> commands) {
    if (!Collections.disjoint(commands, COMPOSED_COMMANDS)) {
      throw new DTIllegalArgumentException(commands,
                                           "composed commands may not be scheduled or added to another composition");
    }
  }

  /**
   * Adds a callback to run automatically every cycle.
   *
   * @param r
   *        the task to run
   */
  public static void bindCallback(Runnable r) {
    CALLBACKS.add(r);
  }
}
