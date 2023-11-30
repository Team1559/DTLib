package org.victorrobotics.dtlib.command;

/**
 * A command that schedules another command when initialized, and ends when it
 * ends. If this command is interrupted, it will cancel the command. Useful for
 * forking off from compositions without retaining requirements.
 * <p>
 * The rules for command compositions do NOT apply, which must be taken into
 * consideration.
 */
public class ProxyCommand extends Command {
  private final Command target;

  /**
   * Constructs a new ProxyCommand.
   *
   * @param target the command to schedule upon execution
   */
  public ProxyCommand(Command target) {
    this.target = target;
  }

  @Override
  public void initialize() {
    target.schedule();
  }

  @Override
  public void execute() {
    // Target is already scheduled
  }

  @Override
  public void end() {
    // If called, target must be finished, and end() will be called naturally
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
