package org.victorrobotics.dtlib.command;

/**
 * A command that schedules another command when initialized, and ends when it
 * ends. If this command is interrupted, it will cancel the command. Useful for
 * forking off from compositions without retaining requirements.
 * <p>
 * The rules for command compositions do NOT apply, which must be taken into
 * consideration.
 */
public class ProxyCommand extends CommandBase {
  private final Command target;

  /**
   * Constructs a new DTProxyCommand.
   *
   * @param target
   *        the command to schedule upon execution
   */
  public ProxyCommand(Command target) {
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
  public ProxyCommand proxy() {
    return this;
  }
}
