package org.victorrobotics.dtlib.subsystem.swerve;

import org.victorrobotics.dtlib.command.Command;

import java.util.function.DoubleSupplier;

public class SwerveTeleopCommand extends Command {
  private final SwerveDrive swerve;

  private final boolean isFieldRelative;

  private final DoubleSupplier vx;
  private final DoubleSupplier vy;
  private final DoubleSupplier vr;

  public SwerveTeleopCommand(SwerveDrive swerve, boolean isFieldRelative, DoubleSupplier vx,
                             DoubleSupplier vy, DoubleSupplier vr) {
    this.swerve = swerve;
    this.isFieldRelative = isFieldRelative;
    this.vx = vx;
    this.vy = vy;
    this.vr = vr;
    
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    swerve.configFieldRelative(isFieldRelative);
  }

  @Override
  public void execute() {
    swerve.driveVelocity(vx.getAsDouble(), vy.getAsDouble(), vr.getAsDouble());
  }

  @Override
  public void end() {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
