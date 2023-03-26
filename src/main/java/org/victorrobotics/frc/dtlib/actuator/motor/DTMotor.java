package org.victorrobotics.frc.dtlib.actuator.motor;

import org.victorrobotics.frc.dtlib.function.supplier.DTLimitedBooleanSupplier;
import org.victorrobotics.frc.dtlib.function.supplier.DTLimitedDoubleSupplier;
import org.victorrobotics.frc.dtlib.function.supplier.DTLimitedLongSupplier;
import org.victorrobotics.frc.dtlib.function.supplier.DTLimitedSupplier;
import org.victorrobotics.frc.dtlib.network.DTSendable;

import edu.wpi.first.util.sendable.SendableBuilder;

public interface DTMotor<MOTORTYPE, CURRENTLIMITTYPE> extends DTSendable, AutoCloseable {
    @Override
    void close();

    @Override
    default void initSendable(SendableBuilder builder) {
        builder.setActuator(true);
        builder.setSafeState(this::neutralOutput);

        builder.addIntegerProperty("CAN ID",
                new DTLimitedLongSupplier(this::getCanID, UPDATE_RATE_SLOW_HZ), null);

        builder.addDoubleProperty("Output", this::getMotorOutputPercent, this::setPercentOutput);
        builder.addDoubleProperty("Position", this::getEncoderPosition, this::setPosition);
        builder.addDoubleProperty("Velocity", this::getEncoderVelocity, this::setVelocity);

        builder.addBooleanProperty("Brake mode",
                new DTLimitedBooleanSupplier(this::isBrakeEnabled, UPDATE_RATE_SLOW_HZ),
                this::configBrakeMode);
        builder.addBooleanProperty("Inverted",
                new DTLimitedBooleanSupplier(this::isOutputInverted, UPDATE_RATE_SLOW_HZ),
                this::configOutputInverted);

        builder.addDoubleProperty("kP",
                new DTLimitedDoubleSupplier(this::getPIDproportional, UPDATE_RATE_SLOW_HZ),
                this::configPIDproportional);
        builder.addDoubleProperty("kI",
                new DTLimitedDoubleSupplier(this::getPIDintegral, UPDATE_RATE_SLOW_HZ),
                this::configPIDintegral);
        builder.addDoubleProperty("kD",
                new DTLimitedDoubleSupplier(this::getPIDderivative, UPDATE_RATE_SLOW_HZ),
                this::configPIDderivative);
        builder.addDoubleProperty("kF",
                new DTLimitedDoubleSupplier(this::getPIDfeedforward, UPDATE_RATE_SLOW_HZ),
                this::configPIDfeedforward);
        builder.addDoubleProperty("kIZ",
                new DTLimitedDoubleSupplier(this::getPIDintegralZone, UPDATE_RATE_SLOW_HZ),
                this::configPIDintegralZone);

        builder.addDoubleProperty("Voltage",
                new DTLimitedDoubleSupplier(this::getInputVoltage, UPDATE_RATE_STD_HZ), null);
        builder.addDoubleProperty("Temperature",
                new DTLimitedDoubleSupplier(this::getTemperature, UPDATE_RATE_SLOW_HZ), null);
        builder.addBooleanProperty("Fault",
                new DTLimitedBooleanSupplier(() -> getFaults().hasAnyFault(), UPDATE_RATE_STD_HZ),
                null);
        builder.addStringProperty("Firmware",
                new DTLimitedSupplier<>(this::getFirmwareVersion, UPDATE_RATE_SLOW_HZ), null);

        customizeSendable(builder);
    }

    default void customizeSendable(SendableBuilder builder) {}

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
