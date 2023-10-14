package org.victorrobotics.dtlib.subsystem;

import org.victorrobotics.dtlib.command.Command;
import org.victorrobotics.dtlib.command.WrapperCommand;

import java.util.HashMap;
import java.util.Map;

// import edu.wpi.first.wpilibj2.command.Subsystem;

public class WPILibSubsystem extends Subsystem {
  private static final Map<edu.wpi.first.wpilibj2.command.Subsystem, WPILibSubsystem> INSTANCES = new HashMap<>();

  private final edu.wpi.first.wpilibj2.command.Subsystem internal;

  protected WPILibSubsystem(edu.wpi.first.wpilibj2.command.Subsystem internal) {
    this.internal = internal;
  }

  @Override
  public void periodic() {
    internal.periodic();
  }

  @Override
  public void simulationPeriodic() {
    internal.simulationPeriodic();
  }

  @Override
  public void setDefaultCommand(Command command) {
    if (command instanceof WrapperCommand wrapper) {
      internal.setDefaultCommand(wrapper.getWPILibCommand());
      super.setDefaultCommand(command);
    } else {
      throw new UnsupportedOperationException("Cannot convert " + command + " to WPILib command");
    }
  }

  public static WPILibSubsystem of(edu.wpi.first.wpilibj2.command.Subsystem subsystem) {
    return INSTANCES.computeIfAbsent(subsystem, WPILibSubsystem::new);
  }

  @Override
  public void close() {}
}
