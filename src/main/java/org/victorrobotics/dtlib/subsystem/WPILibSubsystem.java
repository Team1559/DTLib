package org.victorrobotics.dtlib.subsystem;

import java.util.HashMap;
import java.util.Map;

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

  public static WPILibSubsystem of(edu.wpi.first.wpilibj2.command.Subsystem subsystem) {
    return INSTANCES.computeIfAbsent(subsystem, WPILibSubsystem::new);
  }
}
