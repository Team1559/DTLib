package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.subsystem.WPILibSubsystem;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A command that provides a compatibility layer between WPILib's commands and
 * DTLib's commands.
 *
 * @see TargetCommand
 * @see edu.wpi.first.wpilibj2.command.Command
 */
public class WrapperCommand extends Command {
  private final edu.wpi.first.wpilibj2.command.Command target;

  /**
   * Constructs a new WrapperCommand.
   *
   * @param command the command to execute
   */
  public WrapperCommand(edu.wpi.first.wpilibj2.command.Command command) {
    this.target = Objects.requireNonNull(command);
    CommandScheduler.getInstance()
                    .registerComposedCommands(command);

    for (Subsystem subsystem : command.getRequirements()) {
      addRequirements(WPILibSubsystem.of(subsystem));
    }
  }

  /**
   * Gets the corresponding WPILib Command object.
   *
   * @return the WPILib command run by this command
   * @see edu.wpi.first.wpilibj2.command.Command
   */
  public edu.wpi.first.wpilibj2.command.Command getWPILibCommand() {
    return target;
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
  public boolean isFinished() {
    return target.isFinished();
  }

  @Override
  public void end() {
    target.end(false);
  }

  @Override
  public void interrupt() {
    target.end(true);
  }

  @Override
  public boolean runsWhenDisabled() {
    return target.runsWhenDisabled();
  }

  @Override
  public boolean isInterruptible() {
    return target.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf;
  }

  @Override
  public String getName() {
    return target.getName();
  }
}
