// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.victorrobotics.dtlib.controller;

import org.victorrobotics.dtlib.command.DTCommand;
import org.victorrobotics.dtlib.command.DTCommandScheduler;

import java.util.Objects;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;

/**
 * This class provides an easy way to link commands to conditions. It is
 * modified from edu.wpi.first.wpilibj2.command.button.Trigger, maintaining API
 * compatibility.
 */
public class DTTrigger implements BooleanSupplier {
  private final BooleanSupplier condition;

  private boolean value;
  private boolean previous;

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
    DTCommandScheduler.bindCallback(this::refresh);
    refresh();
  }

  private void refresh() {
    previous = value;
    value = condition.getAsBoolean();
  }

  @Override
  public boolean getAsBoolean() {
    return value;
  }

  /**
   * Starts the given command whenever the condition changes from `false` to
   * `true`.
   *
   * @param command
   *        the command to start
   */
  public void onTrue(DTCommand command) {
    Objects.requireNonNull(command);
    DTCommandScheduler.bindCallback(() -> {
      if (!previous && value) {
        command.schedule();
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
  public void onFalse(DTCommand command) {
    Objects.requireNonNull(command);
    DTCommandScheduler.bindCallback(() -> {
      if (previous && !value) {
        command.schedule();
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
  public void whileTrue(DTCommand command) {
    Objects.requireNonNull(command);
    DTCommandScheduler.bindCallback(() -> {
      if (!previous && value) {
        command.schedule();
      } else if (previous && !value) {
        command.cancel();
      }
    });
  }

  /**
   * Starts the given command when the condition changes to `false` and cancels
   * it when the condition changes to `true`.
   * <p>
   * Doesn't re-start the command if it ends while the condition is still
   * `false`. If the command should restart, see
   * {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
   *
   * @param command
   *        the command to start
   */
  public void whileFalse(DTCommand command) {
    Objects.requireNonNull(command);
    DTCommandScheduler.bindCallback(() -> {
      if (previous && !value) {
        command.schedule();
      } else if (!previous && value) {
        command.cancel();
      }
    });
  }

  /**
   * Toggles a command when the condition changes from `false` to `true`.
   *
   * @param command
   *        the command to toggle
   */
  public void toggleOnTrue(DTCommand command) {
    Objects.requireNonNull(command);
    DTCommandScheduler.bindCallback(() -> {
      if (!previous && value) {
        if (command.isScheduled()) {
          command.cancel();
        } else {
          command.schedule();
        }
      }
    });
  }

  /**
   * Toggles a command when the condition changes from `true` to `false`.
   *
   * @param command
   *        the command to toggle
   */
  public void toggleOnFalse(DTCommand command) {
    Objects.requireNonNull(command);
    DTCommandScheduler.bindCallback(() -> {
      if (previous && !value) {
        if (command.isScheduled()) {
          command.cancel();
        } else {
          command.schedule();
        }
      }
    });
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
    return new DTTrigger(() -> value && other.getAsBoolean());
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
    return new DTTrigger(() -> value || other.getAsBoolean());
  }

  /**
   * Composes two conditions with logical XOR.
   *
   * @param other
   *        the condition to compose with
   *
   * @return A DTTrigger which is active when either condition is true, but not
   *         both.
   */
  public DTTrigger xor(BooleanSupplier other) {
    return new DTTrigger(() -> value != other.getAsBoolean());
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
    return new DTTrigger(() -> value && !other.getAsBoolean());
  }

  /**
   * Creates a new DTTrigger that is the logical NOT of this DTTrigger.
   *
   * @return the negated DTTrigger
   */
  public DTTrigger negate() {
    return new DTTrigger(() -> !value);
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
        return debouncer.calculate(value);
      }
    });
  }
}
