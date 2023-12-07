package org.victorrobotics.dtlib.controller;

import org.victorrobotics.dtlib.command.CommandScheduler;
import org.victorrobotics.dtlib.log.LogWriter;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * A base class for controller input from the DriverStation.
 */
public class Controller {
  private static final int MAX_CONTROLLER_COUNT = DriverStation.kJoystickPorts;

  private static int INSTANCES;

  private final int port;

  private final double[] axes;
  private final int[]    povs;
  private int            buttons;

  @SuppressWarnings("java:S3366")
  protected Controller(int port, int axisCount, int povCount) {
    if (port < -1 || port >= MAX_CONTROLLER_COUNT) {
      throw new ArrayIndexOutOfBoundsException(port);
    }
    if ((INSTANCES & (1 << port)) != 0) {
      LogWriter.warn("Controller already instantiated at port " + port);
    }
    this.port = port;
    INSTANCES |= 1 << port;

    axes = new double[axisCount];
    povs = new int[povCount];

    CommandScheduler.bindCallback(this::refresh);
    refresh();
  }

  protected final Trigger getButton(int index) {
    if (index >= 32 || index < 0) {
      throw new ArrayIndexOutOfBoundsException(index);
    }
    return new Trigger(() -> (buttons & (1 << index)) != 0);
  }

  protected final Axis getAxis(int index) {
    if (index >= axes.length || index < 0) {
      throw new ArrayIndexOutOfBoundsException(index);
    }
    return new Axis(() -> axes[index]);
  }

  protected final Pov getPov(int index) {
    if (index >= povs.length || index < 0) {
      throw new ArrayIndexOutOfBoundsException(index);
    }
    return new Pov(() -> povs[index]);
  }

  private void refresh() {
    if (!isConnected()) {
      buttons = 0;
      for (int i = 0; i < axes.length; i++) {
        axes[i] = 0;
      }
      for (int i = 0; i < povs.length; i++) {
        povs[i] = -1;
      }
      return;
    }

    buttons = DriverStation.getStickButtons(port);

    for (int i = 0; i < axes.length; i++) {
      axes[i] = DriverStation.getStickAxis(port, i);
    }

    for (int i = 0; i < povs.length; i++) {
      povs[i] = DriverStation.getStickPOV(port, i);
    }
  }

  public final boolean isConnected() {
    return DriverStation.isJoystickConnected(port);
  }
}
