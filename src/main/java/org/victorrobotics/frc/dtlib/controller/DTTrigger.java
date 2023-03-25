// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.victorrobotics.frc.dtlib.controller;

import java.util.Objects;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This class provides an easy way to link commands to conditions. It is
 * modified from edu.wpi.first.wpilibj2.command.button.Trigger, maintaining API
 * compatibility.
 */
public class DTTrigger implements BooleanSupplier {
    private final BooleanSupplier condition;

    /**
     * Creates a new DTTrigger based on the given condition.
     * <p>
     * Polled by the default scheduler button
     * CommandScheduler.getInstance().getDefaultButtonLoop().
     *
     * @param condition
     *        the condition represented by this DTTrigger
     */
    public DTTrigger(BooleanSupplier condition) {
        this.condition = Objects.requireNonNull(condition);
    }

    /**
     * Starts the given command whenever the condition changes from `false` to
     * `true`.
     *
     * @param command
     *        the command to start
     */
    public void onTrue(Command command) {
        Objects.requireNonNull(command);
        getDefaultButtonLoop().bind(new Runnable() {
            private boolean pressedLast = condition.getAsBoolean();

            @Override
            public void run() {
                boolean pressed = condition.getAsBoolean();
                if (!pressedLast && pressed) {
                    command.schedule();
                }
                pressedLast = pressed;
            }
        });
    }

    /**
     * Starts the given command whenever the condition changes from `true` to
     * `false`.
     *
     * @param command
     *        the command to start
     */
    public void onFalse(Command command) {
        Objects.requireNonNull(command);
        getDefaultButtonLoop().bind(new Runnable() {
            private boolean pressedLast = condition.getAsBoolean();

            @Override
            public void run() {
                boolean pressed = condition.getAsBoolean();
                if (pressedLast && !pressed) {
                    command.schedule();
                }
                pressedLast = pressed;
            }
        });
    }

    /**
     * Starts the given command when the condition changes to `true` and cancels
     * it when the condition changes to `false`.
     * <p>
     * Doesn't re-start the command if it ends while the condition is still
     * `true`. If the command should restart, see
     * {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
     *
     * @param command
     *        the command to start
     */
    public void whileTrue(Command command) {
        Objects.requireNonNull(command);
        getDefaultButtonLoop().bind(new Runnable() {
            private boolean pressedLast = condition.getAsBoolean();

            @Override
            public void run() {
                boolean pressed = condition.getAsBoolean();
                if (!pressedLast && pressed) {
                    command.schedule();
                } else if (pressedLast && !pressed) {
                    command.cancel();
                }
                pressedLast = pressed;
            }
        });
    }

    /**
     * Starts the given command when the condition changes to `false` and
     * cancels it when the condition changes to `true`.
     * <p>
     * Doesn't re-start the command if it ends while the condition is still
     * `false`. If the command should restart, see
     * {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
     *
     * @param command
     *        the command to start
     */
    public void whileFalse(Command command) {
        Objects.requireNonNull(command);
        getDefaultButtonLoop().bind(new Runnable() {
            private boolean pressedLast = condition.getAsBoolean();

            @Override
            public void run() {
                boolean pressed = condition.getAsBoolean();
                if (pressedLast && !pressed) {
                    command.schedule();
                } else if (!pressedLast && pressed) {
                    command.cancel();
                }
                pressedLast = pressed;
            }
        });
    }

    /**
     * Toggles a command when the condition changes from `false` to `true`.
     *
     * @param command
     *        the command to toggle
     */
    public void toggleOnTrue(Command command) {
        Objects.requireNonNull(command);
        getDefaultButtonLoop().bind(new Runnable() {
            private boolean pressedLast = condition.getAsBoolean();

            @Override
            public void run() {
                boolean pressed = condition.getAsBoolean();
                if (!pressedLast && pressed) {
                    if (command.isScheduled()) {
                        command.cancel();
                    } else {
                        command.schedule();
                    }
                }
                pressedLast = pressed;
            }
        });
    }

    /**
     * Toggles a command when the condition changes from `true` to `false`.
     *
     * @param command
     *        the command to toggle
     */
    public void toggleOnFalse(Command command) {
        Objects.requireNonNull(command);
        getDefaultButtonLoop().bind(new Runnable() {
            private boolean pressedLast = condition.getAsBoolean();

            @Override
            public void run() {
                boolean pressed = condition.getAsBoolean();
                if (pressedLast && !pressed) {
                    if (command.isScheduled()) {
                        command.cancel();
                    } else {
                        command.schedule();
                    }
                }
                pressedLast = pressed;
            }
        });
    }

    @Override
    public boolean getAsBoolean() {
        return condition.getAsBoolean();
    }

    /**
     * Composes two conditions with logical AND.
     *
     * @param other
     *        the condition to compose with
     *
     * @return A DTTrigger which is active when both conditions are true.
     */
    public DTTrigger and(BooleanSupplier other) {
        return new DTTrigger(() -> condition.getAsBoolean() && other.getAsBoolean());
    }

    /**
     * Composes two conditions with logical OR.
     *
     * @param other
     *        the condition to compose with
     *
     * @return A DTTrigger which is active when either condition is active.
     */
    public DTTrigger or(BooleanSupplier other) {
        return new DTTrigger(() -> condition.getAsBoolean() || other.getAsBoolean());
    }

    /**
     * Composes two conditions with logical XOR.
     *
     * @param other
     *        the condition to compose with
     *
     * @return A DTTrigger which is active when either condition is true, but
     *         not both.
     */
    public DTTrigger xor(BooleanSupplier other) {
        return new DTTrigger(() -> condition.getAsBoolean() != other.getAsBoolean());
    }

    /**
     * Composes two conditions with logical AND NOT.
     *
     * @param other
     *        the condition to compose with
     *
     * @return A DTTrigger which is active when this condition is true and other
     *         is not.
     */
    public DTTrigger unless(BooleanSupplier other) {
        return new DTTrigger(() -> condition.getAsBoolean() && !other.getAsBoolean());
    }

    /**
     * Creates a new DTTrigger that is the logical NOT of this DTTrigger.
     *
     * @return the negated DTTrigger
     */
    public DTTrigger negate() {
        return new DTTrigger(() -> !condition.getAsBoolean());
    }

    /**
     * Creates a new debounced DTTrigger from this DTTrigger - it will become
     * active when this DTTrigger has been active for longer than the specified
     * period.
     *
     * @param seconds
     *        The debounce period.
     *
     * @return The debounced DTTrigger (rising edges debounced only)
     */
    public DTTrigger debounce(double seconds) {
        return debounce(seconds, Debouncer.DebounceType.kRising);
    }

    /**
     * Creates a new debounced DTTrigger from this DTTrigger - it will become
     * active when this DTTrigger has been active for longer than the specified
     * period.
     *
     * @param seconds
     *        The debounce period.
     * @param type
     *        The debounce type.
     *
     * @return The debounced DTTrigger.
     */
    public DTTrigger debounce(double seconds, Debouncer.DebounceType type) {
        return new DTTrigger(new BooleanSupplier() {
            private final Debouncer debouncer = new Debouncer(seconds, type);

            @Override
            public boolean getAsBoolean() {
                return debouncer.calculate(condition.getAsBoolean());
            }
        });
    }

    private static EventLoop getDefaultButtonLoop() {
        return CommandScheduler.getInstance()
                               .getDefaultButtonLoop();
    }
}
