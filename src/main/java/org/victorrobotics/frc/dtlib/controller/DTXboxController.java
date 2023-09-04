package org.victorrobotics.frc.dtlib.controller;

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

  private static final int AXIS_COUNT   = Axis.values().length;
  private static final int BUTTON_COUNT = Button.values().length;
  private static final int POV_COUNT    = 1;
  private static final int DPAD_INDEX   = 0;

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
    super(port, AXIS_COUNT, BUTTON_COUNT, POV_COUNT);

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

    setAxisDeadband(0.05);
  }

  public double getLeftStickX() {
    return getAxisCurrent(Axis.LEFT_X.value);
  }

  public double getLeftStickXSquared() {
    return getAxisSquared(Axis.LEFT_X.value);
  }

  public double getLeftStickY() {
    return getAxisCurrent(Axis.LEFT_Y.value);
  }

  public double getLeftStickYSquared() {
    return getAxisSquared(Axis.LEFT_Y.value);
  }

  public double getLeftTrigger() {
    return getAxisCurrent(Axis.LEFT_TRIGGER.value);
  }

  public double getLeftTriggerSquared() {
    return getAxisSquared(Axis.LEFT_TRIGGER.value);
  }

  public double getRightTrigger() {
    return getAxisCurrent(Axis.RIGHT_TRIGGER.value);
  }

  public double getRightTriggerSquared() {
    return getAxisSquared(Axis.RIGHT_TRIGGER.value);
  }

  public double getRightStickX() {
    return getAxisCurrent(Axis.RIGHT_X.value);
  }

  public double getRightStickXSquared() {
    return getAxisSquared(Axis.RIGHT_X.value);
  }

  public double getRightStickY() {
    return getAxisCurrent(Axis.RIGHT_Y.value);
  }

  public double getRightStickYSquared() {
    return getAxisSquared(Axis.RIGHT_Y.value);
  }

  public boolean getAButton() {
    return getButtonCurrent(Button.A.value);
  }

  public boolean getAButtonPressed() {
    return getButtonPressed(Button.A.value);
  }

  public boolean getAButtonReleased() {
    return getButtonReleased(Button.A.value);
  }

  public boolean getBButton() {
    return getButtonCurrent(Button.B.value);
  }

  public boolean getBButtonPressed() {
    return getButtonPressed(Button.B.value);
  }

  public boolean getBButtonReleased() {
    return getButtonReleased(Button.B.value);
  }

  public boolean getXButton() {
    return getButtonCurrent(Button.X.value);
  }

  public boolean getXButtonPressed() {
    return getButtonPressed(Button.X.value);
  }

  public boolean getXButtonReleased() {
    return getButtonReleased(Button.X.value);
  }

  public boolean getYButton() {
    return getButtonCurrent(Button.Y.value);
  }

  public boolean getYButtonPressed() {
    return getButtonPressed(Button.Y.value);
  }

  public boolean getYButtonReleased() {
    return getButtonReleased(Button.Y.value);
  }

  public boolean getLeftBumper() {
    return getButtonCurrent(Button.LEFT_BUMPER.value);
  }

  public boolean getLeftBumperPressed() {
    return getButtonPressed(Button.LEFT_BUMPER.value);
  }

  public boolean getLeftBumperReleased() {
    return getButtonReleased(Button.LEFT_BUMPER.value);
  }

  public boolean getRightBumper() {
    return getButtonCurrent(Button.RIGHT_BUMPER.value);
  }

  public boolean getRightBumperPressed() {
    return getButtonPressed(Button.RIGHT_BUMPER.value);
  }

  public boolean getRightBumperReleased() {
    return getButtonReleased(Button.RIGHT_BUMPER.value);
  }

  public boolean getLeftMenuButton() {
    return getButtonCurrent(Button.LEFT_MENU.value);
  }

  public boolean getLeftMenuButtonPressed() {
    return getButtonPressed(Button.LEFT_MENU.value);
  }

  public boolean getLeftMenuButtonReleased() {
    return getButtonReleased(Button.LEFT_MENU.value);
  }

  public boolean getRightMenuButton() {
    return getButtonCurrent(Button.RIGHT_MENU.value);
  }

  public boolean getRightMenuButtonPressed() {
    return getButtonPressed(Button.RIGHT_MENU.value);
  }

  public boolean getRightMenuButtonReleased() {
    return getButtonReleased(Button.RIGHT_MENU.value);
  }

  public boolean getLeftStickButton() {
    return getButtonCurrent(Button.LEFT_STICK.value);
  }

  public boolean getLeftStickButtonPressed() {
    return getButtonPressed(Button.LEFT_STICK.value);
  }

  public boolean getLeftStickButtonReleased() {
    return getButtonReleased(Button.LEFT_STICK.value);
  }

  public boolean getRightStickButton() {
    return getButtonCurrent(Button.RIGHT_STICK.value);
  }

  public boolean getRightStickButtonPressed() {
    return getButtonPressed(Button.RIGHT_STICK.value);
  }

  public boolean getRightStickButtonReleased() {
    return getButtonReleased(Button.RIGHT_STICK.value);
  }

  public int getDpad() {
    return getPovCurrent(DPAD_INDEX);
  }
}
