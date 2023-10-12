package org.victorrobotics.dtlib.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DTDash {
  protected static final NetworkTable DASHBOARD_TABLE = NetworkTableInstance.getDefault()
                                                                            .getTable("/DTDash");

  // No instantiation
  private DTDash() {}

  public static NetworkTable getMainTable() {
    return DASHBOARD_TABLE;
  }
}
