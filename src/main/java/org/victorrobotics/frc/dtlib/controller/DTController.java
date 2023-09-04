package org.victorrobotics.frc.dtlib.controller;

import org.victorrobotics.frc.dtlib.exception.DTIllegalArgumentException;

import java.util.Arrays;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public abstract class DTController {
  private static final int MAX_CONTROLLER_COUNT = 6;

  private static final DTController[] INSTANCES = new DTController[MAX_CONTROLLER_COUNT];

  private final int port;

  private double[] axesCurrent;
  private double[] axesPrevious;

  private boolean[] buttonsCurrent;
  private boolean[] buttonsPrevious;

  private int[] povsCurrent;
  private int[] povsPrevious;

  private double deadband;

  protected DTController(int port, int axisCount, int buttonCount, int povCount) {
    if (port < -1 || port >= MAX_CONTROLLER_COUNT) {
      throw new DTIllegalArgumentException("port must be in range 0-" + (MAX_CONTROLLER_COUNT - 1),
          port);
    }
    if (INSTANCES[port] != null) {
      // TODO: warn
    }
    this.port = port;
    INSTANCES[port] = this;

    if (axisCount > 0) {
      axesCurrent = new double[axisCount];
      axesPrevious = new double[axisCount];
    }

    if (buttonCount > 0) {
      buttonsCurrent = new boolean[buttonCount];
      buttonsPrevious = new boolean[buttonCount];
    }

    if (povCount > 0) {
      povsCurrent = new int[povCount];
      povsPrevious = new int[povCount];
    }

    deadband = 0;
  }

  protected final DTAxis getAxis(int index) {
    return new DTAxis(() -> deadband(axesCurrent[index]));
  }

  protected final double getAxisCurrent(int index) {
    return deadband(axesCurrent[index]);
  }

  protected final double getAxisSquared(int index) {
    return squareKeepSign(deadband(axesCurrent[index]));
  }

  protected final double getAxisChange(int index) {
    return deadband(axesCurrent[index]) - deadband(axesPrevious[index]);
  }

  protected final DTTrigger getButton(int index) {
    return new DTTrigger(() -> buttonsCurrent[index]);
  }

  protected final boolean getButtonCurrent(int index) {
    return buttonsCurrent[index];
  }

  protected final boolean getButtonPressed(int index) {
    return !buttonsPrevious[index] && buttonsCurrent[index];
  }

  protected final boolean getButtonReleased(int index) {
    return buttonsPrevious[index] && !buttonsCurrent[index];
  }

  protected final DTPov getPov(int index) {
    return new DTPov(() -> povsCurrent[index]);
  }

  protected final int getPovCurrent(int index) {
    return povsCurrent[index];
  }

  protected final boolean getPovPressed(int index) {
    return povsPrevious[index] != povsCurrent[index] && povsCurrent[index] != -1;
  }

  protected final boolean getPovReleased(int index) {
    return povsPrevious[index] != povsCurrent[index] && povsPrevious[index] != -1;
  }

  protected void refresh() {
    boolean isConnected = isConnected();

    if (axesCurrent != null) {
      // Swap buffers
      double[] temp = axesPrevious;
      axesPrevious = axesCurrent;
      axesCurrent = temp;

      if (isConnected) {
        // Overwrite with new data
        for (int i = 0; i < axesCurrent.length; i++) {
          axesCurrent[i] = DriverStation.getStickAxis(port, i);
        }
      } else {
        // No input
        Arrays.fill(axesCurrent, 0);
      }
    }

    if (buttonsCurrent != null) {
      // Swap buffers
      boolean[] temp = buttonsPrevious;
      buttonsPrevious = buttonsCurrent;
      buttonsCurrent = temp;

      if (isConnected) {
        // Overwrite with new data
        int buttons = DriverStation.getStickButtons(port);
        for (int i = 0; i < buttonsCurrent.length; i++) {
          buttonsCurrent[i] = (buttons & (1 << i)) != 0;
        }
      } else {
        // No input
        Arrays.fill(buttonsCurrent, false);
      }
    }

    if (povsCurrent != null) {
      // Swap buffers
      int[] temp = povsPrevious;
      povsPrevious = povsCurrent;
      povsCurrent = temp;

      if (isConnected) {
        // Overwrite with new data
        for (int i = 0; i < povsCurrent.length; i++) {
          povsCurrent[i] = DriverStation.getStickPOV(port, i);
        }
      } else {
        // No input
        Arrays.fill(povsCurrent, -1);
      }
    }
  }

  public void setAxisDeadband(double deadband) {
    this.deadband = deadband;
  }

  private double deadband(double input) {
    if (deadband < 1e-3) {
      return input;
    } else if (Math.abs(input) <= deadband) {
      return 0D;
    } else if (input < 0D) {
      return (input + deadband) / (1D - deadband);
    } else {
      return (input - deadband) / (1D - deadband);
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

  private static double squareKeepSign(double d) {
    return Math.copySign(d * d, d);
  }

  public static void refreshAll() {
    for (DTController controller : INSTANCES) {
      controller.refresh();
    }
  }
}
