package org.victorrobotics.frc.dtlib.actuator.motor;

import org.victorrobotics.frc.dtlib.network.DTSendable;

import edu.wpi.first.util.sendable.SendableBuilder;

public interface DTMotor extends DTSendable {
  @Override
  void close();

  @Override
  default void initSendable(SendableBuilder builder) {
    builder.setActuator(true);
    builder.setSafeState(this::neutralOutput);

    builder.addIntegerProperty("CAN ID", limitRate(this::getCanID, UPDATE_RATE_SLOW_HZ), null);

    builder.addDoubleProperty("Output", limitRate(this::getMotorOutputPercent, UPDATE_RATE_FAST_HZ),
        this::setPercentOutput);
    builder.addDoubleProperty("Position", limitRate(this::getEncoderPosition, UPDATE_RATE_FAST_HZ),
        this::setPosition);
    builder.addDoubleProperty("Velocity", limitRate(this::getVelocityRPM, UPDATE_RATE_FAST_HZ),
        this::setVelocity);

    builder.addBooleanProperty("Brake mode", limitRate(this::isBrakeEnabled, UPDATE_RATE_SLOW_HZ),
        this::configBrakeMode);
    builder.addBooleanProperty("Inverted", limitRate(this::isOutputInverted, UPDATE_RATE_SLOW_HZ),
        this::configOutputInverted);

    builder.addDoubleProperty("kP", limitRate(this::getPIDproportional, UPDATE_RATE_SLOW_HZ),
        this::configPIDproportional);
    builder.addDoubleProperty("kI", limitRate(this::getPIDintegral, UPDATE_RATE_SLOW_HZ),
        this::configPIDintegral);
    builder.addDoubleProperty("kD", limitRate(this::getPIDderivative, UPDATE_RATE_SLOW_HZ),
        this::configPIDderivative);
    builder.addDoubleProperty("kF", limitRate(this::getPIDfeedforward, UPDATE_RATE_SLOW_HZ),
        this::configPIDfeedforward);
    builder.addDoubleProperty("kIZ", limitRate(this::getPIDintegralZone, UPDATE_RATE_SLOW_HZ),
        this::configPIDintegralZone);

    builder.addDoubleProperty("Voltage", limitRate(this::getInputVoltage, UPDATE_RATE_STD_HZ),
        null);
    builder.addDoubleProperty("Temperature", limitRate(this::getTemperature, UPDATE_RATE_SLOW_HZ),
        null);
    builder.addBooleanProperty("Fault",
        limitRate(() -> getFaults().hasAnyFault(), UPDATE_RATE_STD_HZ), null);
    builder.addStringProperty("Firmware", limitRate(this::getFirmwareVersion, UPDATE_RATE_SLOW_HZ),
        null);

    customizeSendable(builder);
  }

  default void customizeSendable(SendableBuilder builder) {}

  Object getMotorImpl();

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

  void configCurrentLimit(int maxSupplyCurrent);

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

  double getVelocityRPM();

  DTMotorFaults getFaults();

  String getFirmwareVersion();

  double getMaxVelocity();

  double getStallTorque();
}
