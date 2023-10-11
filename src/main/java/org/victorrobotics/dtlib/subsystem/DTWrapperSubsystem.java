package org.victorrobotics.dtlib.subsystem;

import org.victorrobotics.dtlib.command.DTCommand;
import org.victorrobotics.dtlib.command.DTWrapperCommand;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class DTWrapperSubsystem extends DTSubsystem {
  private static final Map<Subsystem, DTSubsystem> INSTANCES = new HashMap<>();

  private final Subsystem internal;

  protected DTWrapperSubsystem(Subsystem internal) {
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
  public void setDefaultCommand(DTCommand command) {
    if (command instanceof DTWrapperCommand wrapper) {
      internal.setDefaultCommand(wrapper.getWPILibCommand());
      super.setDefaultCommand(command);
    } else {
      throw new UnsupportedOperationException("Cannot convert " + command + " to WPILib command");
    }
  }

  public static DTSubsystem of(Subsystem subsystem) {
    return INSTANCES.computeIfAbsent(subsystem, DTWrapperSubsystem::new);
  }

  @Override
  public void close() {}
}
