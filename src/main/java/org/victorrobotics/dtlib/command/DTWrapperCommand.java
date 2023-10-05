package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.subsystem.DTSubsystem;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DTWrapperCommand extends DTCommandBase {
  private final Command target;

  public DTWrapperCommand(Command command) {
    this.target = Objects.requireNonNull(command);
    CommandScheduler.getInstance()
                    .registerComposedCommands(command);

    for (Subsystem s : command.getRequirements()) {
      if (s instanceof DTSubsystem) {
        addRequirements((DTSubsystem) s);
      } else {
        // TODO: handle Subsystem -> DTSubsystem conversion
      }
    }
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
