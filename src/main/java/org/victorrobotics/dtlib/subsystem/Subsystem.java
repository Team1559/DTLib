package org.victorrobotics.dtlib.subsystem;

import org.victorrobotics.dtlib.command.Command;
import org.victorrobotics.dtlib.command.CommandScheduler;
import org.victorrobotics.dtlib.command.NullCommand;
import org.victorrobotics.dtlib.dashboard.DTDash;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;

public abstract class Subsystem {
  private static final Map<Class<? extends Subsystem>, Integer> SUBSYSTEM_COUUNTS = new HashMap<>();

  private final NetworkTable dashboardTable;
  private final String       identifier;

  private Command defaultCommand;

  protected Subsystem() {
    identifier = getClass().getSimpleName() + "-"
        + SUBSYSTEM_COUUNTS.compute(getClass(), (c, i) -> i == null ? 1 : (i + 1));
    dashboardTable = DTDash.getMainTable()
                           .getSubTable(identifier);
    CommandScheduler.registerSubsystem(this);
    defaultCommand = new NullCommand();
  }

  public abstract void periodic();

  public abstract void simulationPeriodic();

  public final NetworkTable getDashboardTable() {
    return dashboardTable;
  }

  public final Command getDefaultCommand() {
    return defaultCommand;
  }

  public final void setDefaultCommand(Command command) {
    defaultCommand = command;
  }

  public Command getSelfTestCommand() {
    return new NullCommand();
  }

  public String getName() {
    return identifier;
  }
}
