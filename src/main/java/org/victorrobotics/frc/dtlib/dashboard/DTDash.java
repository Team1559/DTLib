package org.victorrobotics.frc.dtlib.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import javax.swing.text.html.parser.DTD;

public class DTDash {
  protected static final NetworkTable DASHBOARD_TABLE = NetworkTableInstance.getDefault()
                                                                            .getTable("/DTDash");

  // No instantiation
  private DTDash() {}

  public static NetworkTable getMainTable() {
    return DASHBOARD_TABLE;
  }
}
