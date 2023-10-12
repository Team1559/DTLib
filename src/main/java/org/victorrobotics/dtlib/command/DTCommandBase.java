package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.subsystem.DTSubsystem;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;

/**
 * A base class for commands, with integrated requirement management. Custom
 * commands may choose to subclass this class, or implement {@link DTCommand}
 * directly.
 */
public abstract class DTCommandBase implements DTCommand {
  private final Set<DTSubsystem> requirements;
  private final Set<DTSubsystem> unmodifiableReqs;

  /**
   * Constructs a new DTCommandBase
   */
  protected DTCommandBase() {
    requirements = new LinkedHashSet<>();
    unmodifiableReqs = Collections.unmodifiableSet(requirements);
  }

  @Override
  public final Set<DTSubsystem> getRequirements() {
    return unmodifiableReqs;
  }

  /**
   * Mark subsystems as requirements for this command. Commands are given
   * exclusive access to their requirements while they are scheduled: if another
   * command tries to schedule at the same time, one of them will be canceled.
   *
   * @param requirements
   *        the subsystems to add
   */
  public final void addRequirements(DTSubsystem... requirements) {
    for (DTSubsystem subsystem : requirements) {
      this.requirements.add(subsystem);
    }
  }

  /**
   * Mark subsystems as requirements for this command. Commands are given
   * exclusive access to their requirements while they are scheduled: if another
   * command tries to schedule at the same time, one of them will be canceled.
   *
   * @param requirements
   *        the set of subsystems to add
   */
  public final void addRequirements(Set<DTSubsystem> requirements) {
    this.requirements.addAll(requirements);
  }
}
