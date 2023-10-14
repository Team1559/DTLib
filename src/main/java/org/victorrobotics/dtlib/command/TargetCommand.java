package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.subsystem.DTSubsystem;

import java.util.Set;

/**
 * <p>
 * The rules for command compositions apply: command instances that are passed
 * to it cannot be added to any other composition or scheduled individually, and
 * the composition requires all subsystems its components require.
 */
public class TargetCommand implements Command {
  protected final Command target;

  public TargetCommand(Command target) {
    this.target = target;
    CommandScheduler.registerComposed(target);
  }

  @Override
  public void initialize() {
    target.initialize();
  }

  @Override
  public void execute() {
    target.execute();
  }

  @Override
  public void end() {
    target.end();
  }

  @Override
  public void interrupt() {
    target.interrupt();
  }

  @Override
  public boolean isFinished() {
    return target.isFinished();
  }

  @Override
  public boolean wasSuccessful() {
    return target.wasSuccessful();
  }

  @Override
  public Set<DTSubsystem> getRequirements() {
    return target.getRequirements();
  }

  @Override
  public boolean isInterruptible() {
    return target.isInterruptible();
  }

  @Override
  public boolean runsWhenDisabled() {
    return target.runsWhenDisabled();
  }

  @Override
  public String getName() {
    return target.getName();
  }
}
