package org.victorrobotics.dtlib.command;

public class DTProxyCommand extends DTCommandBase {
  private final DTCommand target;

  public DTProxyCommand(DTCommand target) {
    this.target = target;
  }

  @Override
  public void initialize() {
    target.schedule();
  }

  @Override
  public void interrupt() {
    target.cancel();
  }

  @Override
  public boolean isFinished() {
    return !target.isScheduled();
  }

  @Override
  public boolean runsWhenDisabled() {
    return target.runsWhenDisabled();
  }

  @Override
  public DTProxyCommand proxy() {
    return this;
  }
}
