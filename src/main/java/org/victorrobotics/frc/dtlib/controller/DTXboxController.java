package org.victorrobotics.frc.dtlib.controller;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;

public class DTXboxController {
    public enum RumbleSide {
        LEFT(true, false),
        RIGHT(false, true),
        BOTH(true, true);

        private final boolean isLeft;
        private final boolean isRight;

        RumbleSide(boolean isLeft, boolean isRight) {
            this.isLeft = isLeft;
            this.isRight = isRight;
        }
    }

    private class RumbleTask extends TimerTask {
        @Override
        public void run() {
            long currentTime = System.currentTimeMillis();
            if (currentTime >= leftRumbleTimeout) {
                leftRumblePower = 0;
            }
            if (currentTime >= rightRumbleTimeout) {
                rightRumblePower = 0;
            }
            controller.setRumble(RumbleType.kLeftRumble, leftRumblePower);
            controller.setRumble(RumbleType.kRightRumble, rightRumblePower);
        }
    }

    private static final Timer  RUMBLE_TIMER   = new Timer("Xbox_Rumble");
    private static final double AXIS_DEADBAND  = 0.05;

    private final XboxController controller;

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

    public final DTTrigger dpadNone;
    public final DTTrigger dpadUp;
    public final DTTrigger dpadUpRight;
    public final DTTrigger dpadRight;
    public final DTTrigger dpadDownRight;
    public final DTTrigger dpadDown;
    public final DTTrigger dpadDownLeft;
    public final DTTrigger dpadLeft;
    public final DTTrigger dpadUpLeft;

    public final DTAxis leftStickX;
    public final DTAxis leftStickY;
    public final DTAxis rightStickX;
    public final DTAxis rightStickY;
    public final DTAxis leftTrigger;
    public final DTAxis rightTrigger;

    private double leftRumblePower;
    private double rightRumblePower;
    private long   leftRumbleTimeout;
    private long   rightRumbleTimeout;
    private double axisDeadBand;

    public DTXboxController(int port) {
        controller = new XboxController(port);
        axisDeadBand = AXIS_DEADBAND;
        RUMBLE_TIMER.scheduleAtFixedRate(new RumbleTask(), 10, 20);

        aButton = new DTTrigger(controller::getAButton);
        bButton = new DTTrigger(controller::getBButton);
        xButton = new DTTrigger(controller::getXButton);
        yButton = new DTTrigger(controller::getYButton);
        leftMenuButton = new DTTrigger(controller::getBackButton);
        rightMenuButton = new DTTrigger(controller::getStartButton);
        leftBumper = new DTTrigger(controller::getLeftBumper);
        rightBumper = new DTTrigger(controller::getRightBumper);
        leftStickButton = new DTTrigger(controller::getLeftStickButton);
        rightStickButton = new DTTrigger(controller::getRightStickButton);

        dpadNone = new DTTrigger(() -> controller.getPOV() == -1);
        dpadUp = new DTTrigger(() -> controller.getPOV() == 0);
        dpadUpRight = new DTTrigger(() -> controller.getPOV() == 45);
        dpadRight = new DTTrigger(() -> controller.getPOV() == 90);
        dpadDownRight = new DTTrigger(() -> controller.getPOV() == 135);
        dpadDown = new DTTrigger(() -> controller.getPOV() == 180);
        dpadDownLeft = new DTTrigger(() -> controller.getPOV() == 225);
        dpadLeft = new DTTrigger(() -> controller.getPOV() == 270);
        dpadUpLeft = new DTTrigger(() -> controller.getPOV() == 315);

        leftStickX = new DTAxis(controller::getLeftX);
        leftStickY = new DTAxis(controller::getLeftY);
        rightStickX = new DTAxis(controller::getRightX);
        rightStickY = new DTAxis(controller::getRightY);
        leftTrigger = new DTAxis(controller::getLeftTriggerAxis);
        rightTrigger = new DTAxis(controller::getRightTriggerAxis);
    }

    public int getDpad() {
        return controller.getPOV();
    }

    /**
     * Sets the rumble on the controller
     *
     * @param duration
     *        Time in seconds for the rumble to last
     */
    public void startRumble(double duration) {
        startRumble(duration, 1);
    }

    /**
     * Sets the rumble on the controller
     *
     * @param duration
     *        Time in seconds for the rumble to last
     * @param side
     *        What side the ruble on <code>LEFT<code>, <code>RIGHT<code>,
     *        <code>BOTH<code>
     */
    public void startRumble(double duration, RumbleSide side) {
        startRumble(duration, 1, side);
    }

    /**
     * Sets the rumble on the controller
     *
     * @param duration
     *        Time in seconds for the rumble to last
     * @param power
     *        Strength of rumble. Values range from 0-1
     */
    public void startRumble(double duration, double power) {
        startRumble(duration, power, RumbleSide.BOTH);
    }

    /**
     * Sets the rumble on the controller
     *
     * @param duration
     *        Time in seconds for the rumble to last
     * @param power
     *        Strength of rumble. Values range from 0-1
     * @param side
     *        What side the ruble on <code>LEFT<code>, <code>RIGHT<code>,
     *        <code>BOTH<code>
     */
    public void startRumble(double duration, double power, RumbleSide side) {
        long timeout = (long) (1000 * duration) + System.currentTimeMillis();
        if (side.isLeft) {
            leftRumblePower = power;
            leftRumbleTimeout = timeout;
        }
        if (side.isRight) {
            rightRumblePower = power;
            rightRumbleTimeout = timeout;
        }
    }

    /**
     * Stops the rumble on both sides
     */
    public void stopRumble() {
        stopRumble(RumbleSide.BOTH);
    }

    /**
     * Stops the rumble on a certain side
     *
     * @param side
     *        What side to stop the ruble on <code>LEFT<code>,
     *        <code>RIGHT<code>, <code>BOTH<code>
     */
    public void stopRumble(RumbleSide side) {
        if (side.isLeft) {
            leftRumbleTimeout = Long.MIN_VALUE;
        }
        if (side.isRight) {
            rightRumbleTimeout = Long.MIN_VALUE;
        }
    }

    public boolean getAButton() {
        return controller.getAButton();
    }

    public boolean getBButton() {
        return controller.getBButton();
    }

    public boolean getXButton() {
        return controller.getXButton();
    }

    public boolean getYButton() {
        return controller.getYButton();
    }

    public boolean getStartButton() {
        return controller.getStartButton();
    }

    public boolean getBackButton() {
        return controller.getBackButton();
    }

    public boolean getLeftStickButton() {
        return controller.getLeftStickButton();
    }

    public boolean getRightStickButton() {
        return controller.getRightStickButton();
    }

    public boolean getLeftBumper() {
        return controller.getLeftBumper();
    }

    public boolean getRightBumper() {
        return controller.getRightBumper();
    }

    public boolean getAButtonPressed() {
        return controller.getAButtonPressed();
    }

    public boolean getBButtonPressed() {
        return controller.getBButtonPressed();
    }

    public boolean getXButtonPressed() {
        return controller.getXButtonPressed();
    }

    public boolean getYButtonPressed() {
        return controller.getYButtonPressed();
    }

    public boolean getStartButtonPressed() {
        return controller.getStartButtonPressed();
    }

    public boolean getBackButtonPressed() {
        return controller.getBackButtonPressed();
    }

    public boolean getLeftStickButtonPressed() {
        return controller.getLeftStickButtonPressed();
    }

    public boolean getRightStickButtonPressed() {
        return controller.getRightStickButtonPressed();
    }

    public boolean getLeftBumperPressed() {
        return controller.getLeftBumperPressed();
    }

    public boolean getRightBumperPressed() {
        return controller.getRightBumperPressed();
    }

    public boolean getAButtonReleased() {
        return controller.getAButtonReleased();
    }

    public boolean getBButtonReleased() {
        return controller.getBButtonReleased();
    }

    public boolean getXButtonReleased() {
        return controller.getXButtonReleased();
    }

    public boolean getYButtonReleased() {
        return controller.getYButtonReleased();
    }

    public boolean getStartButtonReleased() {
        return controller.getStartButtonReleased();
    }

    public boolean getBackButtonReleased() {
        return controller.getBackButtonReleased();
    }

    public boolean getLeftStickButtonReleased() {
        return controller.getLeftStickButtonReleased();
    }

    public boolean getRightStickButtonReleased() {
        return controller.getRightStickButtonReleased();
    }

    public boolean getLeftBumperReleased() {
        return controller.getLeftBumperReleased();
    }

    public boolean getRightBumperReleased() {
        return controller.getRightBumperReleased();
    }

    public double getLeftStickX() {
        return deadband(controller.getLeftX());
    }

    public double getLeftStickY() {
        return deadband(-controller.getLeftY());
    }

    public double getRightStickX() {
        return deadband(controller.getRightX());
    }

    public double getRightStickY() {
        return deadband(-controller.getRightY());
    }

    public double getLeftTrigger() {
        return deadband(controller.getLeftTriggerAxis());
    }

    public double getRightTrigger() {
        return deadband(controller.getRightTriggerAxis());
    }

    public double getLeftStickXSquared() {
        return squareKeepSign(getLeftStickX());
    }

    public double getLeftStickYSquared() {
        return squareKeepSign(getLeftStickY());
    }

    public double getRightStickXSquared() {
        return squareKeepSign(getRightStickX());
    }

    public double getRightStickYSquared() {
        return squareKeepSign(getRightStickY());
    }

    public double getLeftTriggerSquared() {
        return squareKeepSign(getLeftTrigger());
    }

    public double getRightTriggerSquared() {
        return squareKeepSign(getRightTrigger());
    }

    public void setAxisDeadBand(double deadBand) {
        if (!Double.isFinite(deadBand) || axisDeadBand < 0 || axisDeadBand >= 1) {
            throw new IllegalArgumentException("Deadband must satisfy 0 ≤ d < 1");
        } else if (deadBand < 1e-6) {
            axisDeadBand = Double.NaN;
        } else {
            axisDeadBand = deadBand;
        }
    }

    private double deadband(double d) {
        if (Double.isNaN(axisDeadBand)) {
            // Not set, "fast path"
            return d;
        } else if (Math.abs(d) <= axisDeadBand) {
            return 0D;
        } else if (d < 0D) {
            return (d + axisDeadBand) / (1D - axisDeadBand);
        } else {
            return (d - axisDeadBand) / (1D - axisDeadBand);
        }
    }

    private static double squareKeepSign(double d) {
        return Math.copySign(d * d, d);
    }
}
