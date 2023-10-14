package org.victorrobotics.dtlib.controller;

public class XboxController extends Controller {
  private enum XboxAxis {
    LEFT_X(0),
    LEFT_Y(1),
    LEFT_TRIGGER(2),
    RIGHT_TRIGGER(3),
    RIGHT_X(4),
    RIGHT_Y(5);

    private final int value;

    XboxAxis(int value) {
      this.value = value;
    }
  }

  private enum XboxButton {
    A(0),
    B(1),
    X(2),
    Y(3),
    LEFT_BUMPER(4),
    RIGHT_BUMPER(5),
    LEFT_MENU(6),
    RIGHT_MENU(7),
    LEFT_STICK(8),
    RIGHT_STICK(9);

    private final int value;

    XboxButton(int value) {
      this.value = value;
    }
  }

  private static final int AXIS_COUNT = XboxAxis.values().length;
  private static final int POV_COUNT  = 1;
  private static final int DPAD_INDEX = 0;

  public final Trigger aButton;
  public final Trigger bButton;
  public final Trigger xButton;
  public final Trigger yButton;
  public final Trigger leftBumper;
  public final Trigger rightBumper;
  public final Trigger leftMenuButton;
  public final Trigger rightMenuButton;
  public final Trigger leftStickButton;
  public final Trigger rightStickButton;

  public final Axis leftStickX;
  public final Axis leftStickY;
  public final Axis rightStickX;
  public final Axis rightStickY;
  public final Axis leftTrigger;
  public final Axis rightTrigger;

  public final Pov dpad;

  public XboxController(int port) {
    super(port, AXIS_COUNT, POV_COUNT);

    aButton = getButton(XboxButton.A.value);
    bButton = getButton(XboxButton.B.value);
    xButton = getButton(XboxButton.X.value);
    yButton = getButton(XboxButton.Y.value);
    leftBumper = getButton(XboxButton.LEFT_BUMPER.value);
    rightBumper = getButton(XboxButton.RIGHT_BUMPER.value);
    leftMenuButton = getButton(XboxButton.LEFT_MENU.value);
    rightMenuButton = getButton(XboxButton.RIGHT_MENU.value);
    leftStickButton = getButton(XboxButton.LEFT_STICK.value);
    rightStickButton = getButton(XboxButton.RIGHT_STICK.value);

    leftStickX = getAxis(XboxAxis.LEFT_X.value);
    leftStickY = getAxis(XboxAxis.LEFT_Y.value);
    rightStickX = getAxis(XboxAxis.RIGHT_X.value);
    rightStickY = getAxis(XboxAxis.RIGHT_Y.value);
    leftTrigger = getAxis(XboxAxis.LEFT_TRIGGER.value);
    rightTrigger = getAxis(XboxAxis.RIGHT_TRIGGER.value);

    dpad = getPov(DPAD_INDEX);
  }
}
