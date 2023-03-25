package org.victorrobotics.frc.dtlib.actuator.motor;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public interface DTMotor<MOTORTYPE, CURRENTLIMITTYPE> extends Sendable, AutoCloseable {
    @Override
    void close();

    @Override
    default void initSendable(SendableBuilder builder) {
        
    }

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
