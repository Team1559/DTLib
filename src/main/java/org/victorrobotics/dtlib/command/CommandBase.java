package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.subsystem.Subsystem;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;

/**
 * A base class for commands, with integrated requirement management. Custom
 * commands may choose to subclass this class, or implement {@link Command}
 * directly.
 */
public abstract class CommandBase implements Command {
  private final Set<Subsystem> requirements;
  private final Set<Subsystem> unmodifiableReqs;

  /**
   * Constructs a new DTCommandBase
   */
  protected CommandBase() {
    requirements = new LinkedHashSet<>();
    unmodifiableReqs = Collections.unmodifiableSet(requirements);
  }

  @Override
  public final Set<Subsystem> getRequirements() {
    return unmodifiableReqs;
  }

  /**
   * Mark subsystems as requirements for this command. Commands are given
   * exclusive access to their requirements while they are scheduled: if another
   * command tries to schedule at the same time, one of them will be canceled.
   *
   * @param requirements the subsystems to add
   */
  public final void addRequirements(Subsystem... requirements) {
    for (Subsystem subsystem : requirements) {
      this.requirements.add(subsystem);
    }
  }

  /**
   * Mark subsystems as requirements for this command. Commands are given
   * exclusive access to their requirements while they are scheduled: if another
   * command tries to schedule at the same time, one of them will be canceled.
   *
   * @param requirements the set of subsystems to add
   */
  public final void addRequirements(Set<Subsystem> requirements) {
    this.requirements.addAll(requirements);
  }
}
