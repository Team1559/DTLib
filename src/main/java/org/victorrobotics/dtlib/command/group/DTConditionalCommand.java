package org.victorrobotics.frc.dtlib.command.group;

import org.victorrobotics.frc.dtlib.command.DTCommand;
import org.victorrobotics.frc.dtlib.command.DTCommandBase;
import org.victorrobotics.frc.dtlib.command.DTCommandScheduler;

import java.util.function.BooleanSupplier;

public class DTConditionalCommand extends DTCommandBase {
  private final DTCommand       trueCommand;
  private final DTCommand       falseCommand;
  private final BooleanSupplier condition;

  private DTCommand activeCommand;

  public DTConditionalCommand(DTCommand onTrue, DTCommand onFalse, BooleanSupplier condition) {
    this.trueCommand = onTrue;
    this.falseCommand = onFalse;
    this.condition = condition;

    requirements.addAll(trueCommand.getRequirements());
    requirements.addAll(falseCommand.getRequirements());
    DTCommandScheduler.registerComposedCommands(onTrue, onFalse);

    activeCommand = falseCommand;
  }

  @Override
  public void initialize() {
    activeCommand = condition.getAsBoolean() ? trueCommand : falseCommand;
    activeCommand.initialize();
  }

  @Override
  public void execute() {
    activeCommand.execute();
  }

  @Override
  public void end() {
    activeCommand.end();
  }

  @Override
  public void interrupt() {
    activeCommand.interrupt();
  }

  @Override
  public boolean isFinished() {
    return activeCommand.isFinished();
  }

  @Override
  public boolean wasSuccessful() {
    return activeCommand.wasSuccessful();
  }
}
