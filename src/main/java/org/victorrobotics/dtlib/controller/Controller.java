package org.victorrobotics.dtlib.controller;

import org.victorrobotics.dtlib.command.CommandScheduler;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * A base class for command-based controller input from the DriverStation.
 * Buttons and joysticks are represented using
 */
public class DTController {
  private static final int MAX_CONTROLLER_COUNT = DriverStation.kJoystickPorts;

  private static final DTController[] INSTANCES = new DTController[MAX_CONTROLLER_COUNT];

  private final int port;

  private double[] axes;
  private int      buttons;
  private int[]    povs;

  protected DTController(int port, int axisCount, int povCount) {
    if (port < -1 || port >= MAX_CONTROLLER_COUNT) {
      throw new ArrayIndexOutOfBoundsException(port);
    }
    if (INSTANCES[port] != null) {
      DriverStation.reportWarning("Controller already instantiated at port " + port, isConnected());
    }
    this.port = port;
    INSTANCES[port] = this;

    if (axisCount > 0) {
      axes = new double[axisCount];
    }

    if (povCount > 0) {
      povs = new int[povCount];
    }

    CommandScheduler.bindCallback(this::refresh);
    refresh();
  }

  protected final DTTrigger getButton(int index) {
    if (index >= 32 || index < 0) {
      throw new ArrayIndexOutOfBoundsException(index);
    }
    return new DTTrigger(() -> (buttons & (1 << index)) != 0);
  }

  protected final DTAxis getAxis(int index) {
    if (povs == null || index >= axes.length || index < 0) {
      throw new ArrayIndexOutOfBoundsException(index);
    }
    return new DTAxis(() -> axes[index]);
  }

  protected final DTPov getPov(int index) {
    if (povs == null || index >= povs.length || index < 0) {
      throw new ArrayIndexOutOfBoundsException(index);
    }
    return new DTPov(() -> povs[index]);
  }

  protected void refresh() {
    if (!isConnected()) {
      buttons = 0;
      if (axes != null) {
        for (int i = 0; i < axes.length; i++) {
          axes[i] = 0;
        }
      }
      if (povs != null) {
        for (int i = 0; i < povs.length; i++) {
          povs[i] = -1;
        }
      }
      return;
    }

    buttons = DriverStation.getStickButtons(port) >> 1;

    if (axes != null) {
      for (int i = 0; i < axes.length; i++) {
        axes[i] = DriverStation.getStickAxis(port, i);
      }
    }

    if (povs != null) {
      for (int i = 0; i < povs.length; i++) {
        povs[i] = DriverStation.getStickPOV(port, i);
      }
    }
  }

  public final boolean isConnected() {
    return DriverStation.isJoystickConnected(port);
  }

  public final void setRumble(double leftPower, double rightPower) {
    DriverStationJNI.setJoystickOutputs((byte) port, 0, (short) (leftPower * 0xFFFF),
                                        (short) (rightPower * 0xFFFF));
  }

  protected static Translation2d combineAxes(double x, double y) {
    return new Translation2d(x, y);
  }
}
