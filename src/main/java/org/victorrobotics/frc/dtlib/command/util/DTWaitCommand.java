package org.victorrobotics.frc.dtlib.command.util;

import org.victorrobotics.frc.dtlib.command.DTCommandBase;

import edu.wpi.first.wpilibj.RobotController;

public class DTWaitCommand extends DTCommandBase {
  private final long duration;
  private long       endTime;

  public DTWaitCommand(double duration) {
    this.duration = (long) (duration * 1_000_000);
  }

  @Override
  public void initialize() {
    endTime = RobotController.getFPGATime() + duration;
  }

  @Override
  public boolean isFinished() {
    return RobotController.getFPGATime() >= endTime;
  }

  @Override
  public void end() {
    endTime = -1;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
