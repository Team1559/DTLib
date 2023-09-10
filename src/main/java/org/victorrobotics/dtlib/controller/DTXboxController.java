package org.victorrobotics.dtlib.controller;

public class DTXboxController extends DTController {
  private enum Axis {
    LEFT_X(0),
    LEFT_Y(1),
    LEFT_TRIGGER(2),
    RIGHT_TRIGGER(3),
    RIGHT_X(4),
    RIGHT_Y(5);

    private final int value;

    Axis(int value) {
      this.value = value;
    }
  }

  private enum Button {
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

    Button(int value) {
      this.value = value;
    }
  }

  private static final int AXIS_COUNT = Axis.values().length;
  private static final int POV_COUNT  = 1;
  private static final int DPAD_INDEX = 0;

  public final DTTrigger aButton;
  public final DTTrigger bButton;
  public final DTTrigger xButton;
  public final DTTrigger yButton;
  public final DTTrigger leftBumper;
  public final DTTrigger rightBumper;
  public final DTTrigger leftMenuButton;
  public final DTTrigger rightMenuButton;
  public final DTTrigger leftStickButton;
  public final DTTrigger rightStickButton;

  public final DTAxis leftStickX;
  public final DTAxis leftStickY;
  public final DTAxis rightStickX;
  public final DTAxis rightStickY;
  public final DTAxis leftTrigger;
  public final DTAxis rightTrigger;

  public final DTPov dpad;

  public DTXboxController(int port) {
    super(port, AXIS_COUNT, POV_COUNT);

    aButton = getButton(Button.A.value);
    bButton = getButton(Button.B.value);
    xButton = getButton(Button.X.value);
    yButton = getButton(Button.Y.value);
    leftBumper = getButton(Button.LEFT_BUMPER.value);
    rightBumper = getButton(Button.RIGHT_BUMPER.value);
    leftMenuButton = getButton(Button.LEFT_MENU.value);
    rightMenuButton = getButton(Button.RIGHT_MENU.value);
    leftStickButton = getButton(Button.LEFT_STICK.value);
    rightStickButton = getButton(Button.RIGHT_STICK.value);

    leftStickX = getAxis(Axis.LEFT_X.value);
    leftStickY = getAxis(Axis.LEFT_Y.value);
    rightStickX = getAxis(Axis.RIGHT_X.value);
    rightStickY = getAxis(Axis.RIGHT_Y.value);
    leftTrigger = getAxis(Axis.LEFT_TRIGGER.value);
    rightTrigger = getAxis(Axis.RIGHT_TRIGGER.value);

    dpad = getPov(DPAD_INDEX);
  }
}
