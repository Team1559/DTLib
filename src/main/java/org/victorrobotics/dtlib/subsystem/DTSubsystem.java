package org.victorrobotics.dtlib.subsystem;

import org.victorrobotics.dtlib.command.DTCommand;
import org.victorrobotics.dtlib.command.DTCommandScheduler;
import org.victorrobotics.dtlib.command.DTNullCommand;
import org.victorrobotics.dtlib.dashboard.DTDash;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;

public abstract class DTSubsystem implements AutoCloseable {
  private static final Map<Class<? extends DTSubsystem>, Integer> SUBSYSTEM_COUUNTS =
      new HashMap<>();

  private final NetworkTable dashboardTable;
  private final String       identifier;

  private DTCommand defaultCommand;

  protected DTSubsystem() {
    identifier = getClass().getSimpleName() + "-"
        + SUBSYSTEM_COUUNTS.compute(getClass(), (c, i) -> i == null ? 1 : (i + 1));
    dashboardTable = DTDash.getMainTable()
                           .getSubTable(identifier);
    DTCommandScheduler.registerSubsystem(this);

    defaultCommand = new DTNullCommand();
  }

  public final NetworkTable getDashboardTable() {
    return dashboardTable;
  }

  public DTCommand getSelfTestCommand() {
    return new DTNullCommand();
  }

  public String getName() {
    return identifier;
  }

  public void periodic() {}

  public void simulationPeriodic() {}

  public DTCommand getDefaultCommand() {
    return defaultCommand;
  }

  public void setDefaultCommand(DTCommand command) {
    defaultCommand = command;
  }
}
