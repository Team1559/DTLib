package org.victorrobotics.frc.dtlib.controller;

import org.victorrobotics.frc.dtlib.command.DTCommandScheduler;
import org.victorrobotics.frc.dtlib.exception.DTIllegalArgumentException;

import java.util.Arrays;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class DTController {
  private static final int MAX_CONTROLLER_COUNT = 6;

  private static final DTController[] INSTANCES = new DTController[MAX_CONTROLLER_COUNT];

  private final int port;

  private double[] axes;
  private int      buttons;
  private int[]    povs;

  protected DTController(int port, int axisCount, int povCount) {
    if (port < -1 || port >= MAX_CONTROLLER_COUNT) {
      throw new DTIllegalArgumentException(port, "port must be in range 0-" + (MAX_CONTROLLER_COUNT - 1));
    }
    if (INSTANCES[port] != null) {
      // TODO: warn
    }
    this.port = port;
    INSTANCES[port] = this;

    if (axisCount > 0) {
      axes = new double[axisCount];
    }

    if (povCount > 0) {
      povs = new int[povCount];
    }

    DTCommandScheduler.bindInputCallback(this::refresh);
  }

  protected final DTTrigger getButton(int index) {
    return new DTTrigger(() -> (buttons & (1 << index)) != 0);
  }

  protected final DTAxis getAxis(int index) {
    return new DTAxis(() -> axes[index]);
  }

  protected final DTPov getPov(int index) {
    return new DTPov(() -> povs[index]);
  }

  protected void refresh() {
    if (!isConnected()) {
      buttons = 0;
      if (axes != null) {
        Arrays.fill(axes, 0);
      }
      if (povs != null) {
        Arrays.fill(povs, -1);
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
