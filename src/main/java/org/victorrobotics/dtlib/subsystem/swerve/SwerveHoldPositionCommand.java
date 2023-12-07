package org.victorrobotics.dtlib.subsystem.swerve;

import org.victorrobotics.dtlib.command.Command;

public class SwerveHoldPositionCommand extends Command {
  private final SwerveDrive swerve;

  public SwerveHoldPositionCommand(SwerveDrive swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swerve.holdPosition();
  }

  @Override
  public void end() {}

  @Override
  public boolean isFinished() {
    return false;
  }

}
