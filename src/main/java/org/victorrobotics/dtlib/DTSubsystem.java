package org.victorrobotics.dtlib;

import org.victorrobotics.dtlib.command.DTCommand;
import org.victorrobotics.dtlib.dashboard.DTDash;
import org.victorrobotics.dtlib.exception.DTIllegalArgumentException;
import org.victorrobotics.dtlib.network.DTSendable;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DTSubsystem extends SubsystemBase implements DTSendable {
  private static final Set<DTHardwareComponent>                   ALL_COMPONENTS   = new HashSet<>();
  private static final Map<Class<? extends DTSubsystem>, Integer> SUBSYSTEM_COUNTS = new HashMap<>();

  private final NetworkTable dashboardTable;
  private final String       typename;
  private final String       identifier;
  private final int          id;

  private final Set<DTHardwareComponent> components;

  protected DTSubsystem() {
    Class<? extends DTSubsystem> clazz = getClass();
    typename = clazz.getSimpleName();
    id = SUBSYSTEM_COUNTS.compute(clazz, (c, i) -> i == null ? 1 : (i + 1));

    identifier = typename + "-" + id;
    setName(identifier);
    setSubsystem(typename);

    dashboardTable = DTDash.getMainTable()
                           .getSubTable(identifier);
    components = new HashSet<>();
  }

  public NetworkTable getDashboardTable() {
    return dashboardTable;
  }

  public String getIdentifier() {
    return identifier;
  }

  protected final void addComponent(DTHardwareComponent component) {
    if (components.contains(component)) {
      // Already registered here
      return;
    } else if (!ALL_COMPONENTS.add(component)) {
      // Component has already been added to another subsystem
      throw new DTIllegalArgumentException(component, "components should not belong to multiple subsystems");
    }

    components.add(component);
  }

  @Override
  public final void periodic() {
    try {
      update();
    } catch (Exception e) {
      // TODO: what to do here?
    }
  }

  protected void update() {}

  public abstract DTCommand getSelfTestCommand();
}
