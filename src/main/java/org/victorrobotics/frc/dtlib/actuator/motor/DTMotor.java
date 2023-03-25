package org.victorrobotics.frc.dtlib.actuator.motor;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public interface DTMotor<MOTORTYPE, CURRENTLIMITTYPE> extends Sendable, AutoCloseable {
    @Override
    void close();

    @Override
    default void initSendable(SendableBuilder builder) {
        builder.setActuator(true);
        builder.setSafeState(this::neutralOutput);

        builder.addIntegerProperty("CAN ID", this::getCanID, null);

        builder.addDoubleProperty("Output", this::getMotorOutputPercent, this::setPercentOutput);
        builder.addDoubleProperty("Position", this::getEncoderPosition, this::setPosition);
        builder.addDoubleProperty("Velocity", this::getEncoderVelocity, this::setVelocity);
        builder.addBooleanProperty("Brake mode", this::isBrakeEnabled, this::configBrakeMode);
        builder.addBooleanProperty("Inverted", this::isOutputInverted, this::configOutputInverted);

        builder.addDoubleProperty("kP", this::getPIDproportional, this::configPIDproportional);
        builder.addDoubleProperty("kI", this::getPIDintegral, this::configPIDintegral);
        builder.addDoubleProperty("kD", this::getPIDderivative, this::configPIDderivative);
        builder.addDoubleProperty("kF", this::getPIDfeedforward, this::configPIDfeedforward);
        builder.addDoubleProperty("kIZ", this::getPIDintegralZone, this::configPIDintegralZone);

        builder.addDoubleProperty("Voltage", this::getInputVoltage, null);
        builder.addDoubleProperty("Temperature", this::getTemperature, null);
        builder.addBooleanProperty("Fault", () -> getFaults().hasAnyFault(), null);
        builder.addStringProperty("Firmware", this::getFirmwareVersion, null);

        customSendable(builder);
    }

    default void customSendable(SendableBuilder builder) {}

    MOTORTYPE internal();

    int getCanID();

    void configBrakeMode(boolean enable);

    void configOutputInverted(boolean invert);

    void configOpenLoopRampRate(double seconds0To100);

    void configClosedLoopRampRate(double seconds0To100);

    void configPIDproportional(double proportional);

    void configPIDintegral(double integral);

    void configPIDderivative(double derivative);

    void configPIDfeedforward(double feedforward);

    void configPIDintegralZone(double iZone);

    default void configPIDFIZ(double proportional, double integral, double derivative,
            double feedforward, double integralZone) {
        configPIDproportional(proportional);
        configPIDintegral(integral);
        configPIDderivative(derivative);
        configPIDfeedforward(feedforward);
        configPIDintegralZone(integralZone);
    }

    void configCurrentLimit(CURRENTLIMITTYPE limit);

    boolean isOutputInverted();

    boolean isBrakeEnabled();

    double getPIDproportional();

    double getPIDintegral();

    double getPIDderivative();

    double getPIDfeedforward();

    double getPIDintegralZone();

    void setPercentOutput(double percent);

    void setPosition(double position);

    void setVelocity(double velocity);

    void neutralOutput();

    void setEncoderPosition(double position);

    double getMotorOutputPercent();

    double getInputVoltage();

    double getTemperature();

    double getEncoderPosition();

    double getEncoderVelocity();

    DTMotorFaults getFaults();

    String getFirmwareVersion();


}
