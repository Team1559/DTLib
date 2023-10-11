package org.victorrobotics.dtlib.command.util;

import org.victorrobotics.dtlib.DTSubsystem;
import org.victorrobotics.dtlib.command.DTCommand;
import org.victorrobotics.dtlib.command.DTCommandScheduler;

import java.util.Set;

public class DTTargetCommand implements DTCommand {
  protected final DTCommand target;

  public DTTargetCommand(DTCommand target) {
    this.target = target;
    DTCommandScheduler.registerComposedCommands(target);
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
}