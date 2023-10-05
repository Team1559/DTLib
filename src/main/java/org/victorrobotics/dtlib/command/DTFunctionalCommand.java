package org.victorrobotics.dtlib.command;

import org.victorrobotics.dtlib.subsystem.DTSubsystem;

import java.util.function.BooleanSupplier;

public class DTFunctionalCommand extends DTCommandBase {
  protected final Runnable        init;
  protected final Runnable        execute;
  protected final Runnable        onEnd;
  protected final Runnable        onInterrupt;
  protected final BooleanSupplier isFinished;

  public DTFunctionalCommand(Runnable init, Runnable execute, Runnable onEnd, Runnable onInterrupt,
      BooleanSupplier isFinished, DTSubsystem... requirements) {
    addRequirements(requirements);
    this.init = init;
    this.execute = execute;
    this.onEnd = onEnd;
    this.onInterrupt = onInterrupt;
    this.isFinished = isFinished;
  }

  @Override
  public void initialize() {
    init.run();
  }

  @Override
  public void execute() {
    execute.run();
  }

  @Override
  public void end() {
    onEnd.run();
  }

  @Override
  public void interrupt() {
    onInterrupt.run();
  }

  @Override
  public boolean isFinished() {
    return isFinished.getAsBoolean();
  }
}
