// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.victorrobotics.frc.dtlib.command;

import org.victorrobotics.frc.dtlib.DTRobot;
import org.victorrobotics.frc.dtlib.DTSubsystem;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.WeakHashMap;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class DTCommandScheduler {
    private static final Set<DTCommand> COMPOSED_COMMANDS    = Collections.newSetFromMap(new WeakHashMap<>());
    private static final Set<DTCommand> SCHEDULED_COMMANDS   = new LinkedHashSet<>();
    private static final Set<DTCommand> COMMANDS_TO_SCHEDULE = new LinkedHashSet<>();
    private static final Set<DTCommand> COMMANDS_TO_CANCEL   = new LinkedHashSet<>();

    private static final Map<DTSubsystem, DTCommand> REQUIREMENTS = new LinkedHashMap<>();
    private static final Map<DTSubsystem, DTCommand> SUBSYSTEMS   = new LinkedHashMap<>();

    private static final List<Consumer<DTCommand>> INIT_ACTIONS      = new ArrayList<>();
    private static final List<Consumer<DTCommand>> EXECUTE_ACTIONS   = new ArrayList<>();
    private static final List<Consumer<DTCommand>> INTERRUPT_ACTIONS = new ArrayList<>();
    private static final List<Consumer<DTCommand>> FINISH_ACTIONS    = new ArrayList<>();

    // TODO: add button loops to DTTrigger, DTAxis, etc.

    private static boolean schedulerDisabled;
    private static boolean inRunLoop;

    private static Watchdog loopOverrun;

    private DTCommandScheduler() {}

    /**
     * Initializes a given command, adds its requirements to the list, and
     * performs the init actions.
     *
     * @param command
     *        The command to initialize
     * @param requirements
     *        The command requirements
     */
    private static void startCommand(DTCommand command, Set<DTSubsystem> requirements) {
        SCHEDULED_COMMANDS.add(command);
        for (DTSubsystem requirement : requirements) {
            REQUIREMENTS.put(requirement, command);
        }

        try {
            command.initialize();
        } catch (RuntimeException e) {
            handleCommandException(command, e);
        }

        for (Consumer<DTCommand> action : INIT_ACTIONS) {
            try {
                action.accept(command);
            } catch (RuntimeException e) {
                handleCommandException(command, e);
            }
        }

        addLoopOverrunEpoch(command.getName() + ".initialize()");
    }

    private static void schedule(DTCommand command) {
        if (command == null) {
            DriverStation.reportWarning("Tried to schedule a null command", true);
            return;
        } else if (COMPOSED_COMMANDS.contains(command)) {
            DriverStation.reportWarning("Tried to schedule a composed command", true);
            return;
        }

        if (schedulerDisabled || isScheduled(command) || (DriverStation.isDisabled() && !command.runsWhenDisabled())) {
            return;
        }

        if (inRunLoop) {
            COMMANDS_TO_SCHEDULE.add(command);
            return;
        }

        Set<DTSubsystem> requirements = command.getRequirements();
        if (Collections.disjoint(REQUIREMENTS.keySet(), requirements)) {
            startCommand(command, requirements);
            return;
        }

        for (DTSubsystem requirement : requirements) {
            DTCommand requiring = getRequiringCommand(requirement);
            if (requiring != null && !requiring.isInterruptible()) {
                return;
            }
        }
        for (DTSubsystem requirement : requirements) {
            DTCommand requiring = getRequiringCommand(requirement);
            if (requiring != null) {
                cancel(requiring);
            }
        }
        startCommand(command, requirements);
    }

    /**
     * Schedules multiple commands for execution. Does nothing for commands
     * already scheduled.
     *
     * @param commands
     *        the commands to schedule. No-op on null.
     */
    public static void schedule(DTCommand... commands) {
        for (DTCommand command : commands) {
            schedule(command);
        }
    }

    /**
     * Runs a single iteration of the scheduler. The execution occurs in the
     * following order:
     * <p>
     * DTSubsystem periodic methods are called.
     * <p>
     * Button bindings are polled, and new commands are scheduled from them.
     * <p>
     * Currently-scheduled commands are executed.
     * <p>
     * End conditions are checked on currently-scheduled commands, and commands
     * that are finished have their end methods called and are removed.
     * <p>
     * Any subsystems not being used as requirements have their default methods
     * started.
     */
    public static void run() {
        if (schedulerDisabled) {
            return;
        }

        if (loopOverrun != null) {
            loopOverrun.reset();
        }

        for (DTSubsystem subsystem : SUBSYSTEMS.keySet()) {
            subsystem.periodic();
            if (DTRobot.isSimulation()) {
                subsystem.simulationPeriodic();
            }
            addLoopOverrunEpoch(subsystem.getClass(), ".periodic()");
        }

        inRunLoop = true;
        for (Iterator<DTCommand> iterator = SCHEDULED_COMMANDS.iterator(); iterator.hasNext();) {
            DTCommand command = iterator.next();

            if (!command.runsWhenDisabled() && RobotState.isDisabled()) {
                try {
                    command.interrupt();
                } catch (RuntimeException e) {
                    handleCommandException(command, e);
                }
                for (Consumer<DTCommand> action : INTERRUPT_ACTIONS) {
                    action.accept(command);
                }
                REQUIREMENTS.keySet()
                            .removeAll(command.getRequirements());
                iterator.remove();
                addLoopOverrunEpoch(command.getName() + ".end(true)");
                continue;
            }

            try {
                command.execute();
            } catch (RuntimeException e) {
                handleCommandException(command, e);
            }

            for (Consumer<DTCommand> action : EXECUTE_ACTIONS) {
                action.accept(command);
            }
            addLoopOverrunEpoch(command.getName() + ".execute()");

            boolean finished = true;
            try {
                finished = command.isFinished();
            } catch (RuntimeException e) {
                handleCommandException(command, e);
            }

            if (finished) {
                try {
                    command.end();
                } catch (RuntimeException e) {
                    handleCommandException(command, e);
                }

                for (Consumer<DTCommand> action : FINISH_ACTIONS) {
                    action.accept(command);
                }
                iterator.remove();

                REQUIREMENTS.keySet()
                            .removeAll(command.getRequirements());
                addLoopOverrunEpoch(command.getName() + ".end(false)");
            }
        }
        inRunLoop = false;

        for (DTCommand command : COMMANDS_TO_SCHEDULE) {
            schedule(command);
        }
        COMMANDS_TO_SCHEDULE.clear();

        for (DTCommand command : COMMANDS_TO_CANCEL) {
            cancel(command);
        }
        COMMANDS_TO_CANCEL.clear();

        for (Map.Entry<DTSubsystem, DTCommand> subsystemCommand : SUBSYSTEMS.entrySet()) {
            if (!REQUIREMENTS.containsKey(subsystemCommand.getKey()) && subsystemCommand.getValue() != null) {
                schedule(subsystemCommand.getValue());
            }
        }

        if (loopOverrun != null) {
            loopOverrun.disable();
            if (loopOverrun.isExpired()) {
                System.out.println("CommandScheduler loop overrun");
                loopOverrun.printEpochs();
            }
        }
    }

    /**
     * Registers subsystems with the scheduler. This must be called for the
     * subsystem's periodic block to run when the scheduler is run, and for the
     * subsystem's default command to be scheduled. It is recommended to call
     * this from the constructor of your subsystem implementations.
     *
     * @param subsystems
     *        the subsystem to register
     */
    public static void registerSubsystem(DTSubsystem... subsystems) {
        for (DTSubsystem subsystem : subsystems) {
            if (subsystem == null) {
                DriverStation.reportWarning("Tried to register a null subsystem", true);
                continue;
            }
            if (SUBSYSTEMS.containsKey(subsystem)) {
                DriverStation.reportWarning("Tried to register an already-registered subsystem", true);
                continue;
            }
            SUBSYSTEMS.put(subsystem, null);
        }
    }

    /**
     * Un-registers subsystems with the scheduler. The subsystem will no longer
     * have its periodic block called, and will not have its default command
     * scheduled.
     *
     * @param subsystems
     *        the subsystem to un-register
     */
    public static void unregisterSubsystem(DTSubsystem... subsystems) {
        SUBSYSTEMS.keySet()
                  .removeAll(Set.of(subsystems));
    }

    /**
     * Sets the default command for a subsystem. Registers that subsystem if it
     * is not already registered. Default commands will run whenever there is no
     * other command currently scheduled that requires the subsystem. Default
     * commands should be written to never end (i.e. their
     * {@link DTCommand#isFinished()} method should return false), as they would
     * simply be re-scheduled if they do. Default commands must also require
     * their subsystem.
     *
     * @param subsystem
     *        the subsystem whose default command will be set
     * @param defaultCommand
     *        the default command to associate with the subsystem
     */
    public static void setDefaultCommand(DTSubsystem subsystem, DTCommand defaultCommand) {
        if (subsystem == null) {
            DriverStation.reportWarning("Tried to set a default command for a null subsystem", true);
            return;
        } else if (defaultCommand == null) {
            DriverStation.reportWarning("Tried to set a null default command", true);
            return;
        }

        requireNotComposed(defaultCommand);

        if (!defaultCommand.getRequirements()
                           .contains(subsystem)) {
            throw new IllegalArgumentException("Default commands must require their subsystem!");
        }

        if (!defaultCommand.isInterruptible()) {
            DriverStation.reportWarning("Registering a non-interruptible default command!\n"
                    + "This will likely prevent any other commands from requiring this subsystem.", true);
        }

        SUBSYSTEMS.put(subsystem, defaultCommand);
    }

    /**
     * Removes the default command for a subsystem. The current default command
     * will run until another command is scheduled that requires the subsystem,
     * at which point the current default command will not be re-scheduled.
     *
     * @param subsystem
     *        the subsystem whose default command will be removed
     */
    public static void removeDefaultCommand(DTSubsystem subsystem) {
        if (subsystem == null) {
            DriverStation.reportWarning("Tried to remove a default command for a null subsystem", true);
            return;
        }

        SUBSYSTEMS.put(subsystem, null);
    }

    /**
     * Gets the default command associated with this subsystem. Null if this
     * subsystem has no default command associated with it.
     *
     * @param subsystem
     *        the subsystem to inquire about
     *
     * @return the default command associated with the subsystem
     */
    public static DTCommand getDefaultCommand(DTSubsystem subsystem) {
        return SUBSYSTEMS.get(subsystem);
    }

    /**
     * Cancels commands. The scheduler will only call
     * {@link DTCommand#end(boolean)} method of the canceled command with
     * {@code true}, indicating they were canceled (as opposed to finishing
     * normally).
     * <p>
     * Commands will be canceled regardless of {@link InterruptionBehavior
     * interruption behavior}.
     *
     * @param commands
     *        the commands to cancel
     */
    public static void cancel(DTCommand... commands) {
        if (inRunLoop) {
            COMMANDS_TO_CANCEL.addAll(List.of(commands));
            return;
        }

        for (DTCommand command : commands) {
            if (command == null) {
                DriverStation.reportWarning("Tried to cancel a null command", true);
                continue;
            } else if (!isScheduled(command)) {
                continue;
            }

            SCHEDULED_COMMANDS.remove(command);
            REQUIREMENTS.keySet()
                        .removeAll(command.getRequirements());

            try {
                command.interrupt();
            } catch (RuntimeException e) {
                handleCommandException(command, e);
            }

            for (Consumer<DTCommand> action : INTERRUPT_ACTIONS) {
                action.accept(command);
            }
            addLoopOverrunEpoch(command.getName() + ".end(true)");
        }
    }

    /** Cancels all commands that are currently scheduled. */
    public static void cancelAll() {
        // Copy to array to avoid concurrent modification.
        cancel(SCHEDULED_COMMANDS.toArray(DTCommand[]::new));
    }

    /**
     * Whether the given commands are running. Note that this only works on
     * commands that are directly scheduled by the scheduler; it will not work
     * on commands inside compositions, as the scheduler does not see them.
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
        return REQUIREMENTS.get(subsystem);
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
     * Adds an action to perform on the initialization of any command by the
     * scheduler.
     *
     * @param action
     *        the action to perform
     */
    public static void onCommandInitialize(Consumer<DTCommand> action) {
        INIT_ACTIONS.add(requireNonNullParam(action, "action", "onCommandInitialize"));
    }

    /**
     * Adds an action to perform on the execution of any command by the
     * scheduler.
     *
     * @param action
     *        the action to perform
     */
    public static void onCommandExecute(Consumer<DTCommand> action) {
        EXECUTE_ACTIONS.add(requireNonNullParam(action, "action", "onCommandExecute"));
    }

    /**
     * Adds an action to perform on the interruption of any command by the
     * scheduler.
     *
     * @param action
     *        the action to perform
     */
    public static void onCommandInterrupt(Consumer<DTCommand> action) {
        INTERRUPT_ACTIONS.add(requireNonNullParam(action, "action", "onCommandInterrupt"));
    }

    /**
     * Adds an action to perform on the finishing of any command by the
     * scheduler.
     *
     * @param action
     *        the action to perform
     */
    public static void onCommandFinish(Consumer<DTCommand> action) {
        FINISH_ACTIONS.add(requireNonNullParam(action, "action", "onCommandFinish"));
    }

    /**
     * Register commands as composed. An exception will be thrown if these
     * commands are scheduled directly or added to a composition.
     *
     * @param commands
     *        the commands to register
     *
     * @throws IllegalArgumentException
     *         if the given commands have already been composed.
     */
    public static void registerComposedCommands(DTCommand... commands) {
        var commandSet = Set.of(commands);
        requireNotComposed(commandSet);
        COMPOSED_COMMANDS.addAll(commandSet);
    }

    /**
     * Clears the list of composed commands, allowing all commands to be freely
     * used again.
     * <p>
     * WARNING: Using this haphazardly can result in unexpected/undesirable
     * behavior. Do not use this unless you fully understand what you are doing.
     */
    public static void clearComposedCommands() {
        COMPOSED_COMMANDS.clear();
    }

    /**
     * Removes a single command from the list of composed commands, allowing it
     * to be freely used again.
     * <p>
     * WARNING: Using this haphazardly can result in unexpected/undesirable
     * behavior. Do not use this unless you fully understand what you are doing.
     *
     * @param command
     *        the command to remove from the list of grouped commands
     */
    public static void removeComposedCommand(DTCommand command) {
        COMPOSED_COMMANDS.remove(command);
    }

    public static void setLoopOverrunWatchdog(Watchdog watchdog) {
        loopOverrun = watchdog;
    }

    private static void addLoopOverrunEpoch(String name) {
        if (loopOverrun != null) {
            loopOverrun.addEpoch(name);
        }
    }

    private static void addLoopOverrunEpoch(Class<?> clazz, String name) {
        addLoopOverrunEpoch(clazz.getSimpleName() + name);
    }

    private static void requireNotComposed(DTCommand command) {
        if (COMPOSED_COMMANDS.contains(command)) {
            throw new IllegalArgumentException(
                    "Commands that have been composed may not be added to another composition or scheduled individually!");
        }
    }

    private static void requireNotComposed(Collection<DTCommand> commands) {
        if (!Collections.disjoint(commands, COMPOSED_COMMANDS)) {
            throw new IllegalArgumentException(
                    "Commands that have been composed may not be added to another composition or scheduled individually!");
        }
    }

    private static void handleCommandException(DTCommand command, RuntimeException e) {
        // TODO: handle exceptions thrown by commands
    }
}
