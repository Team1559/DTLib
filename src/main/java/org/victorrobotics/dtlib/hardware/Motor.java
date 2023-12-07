package org.victorrobotics.dtlib.hardware;

public interface Motor {
  Object getMotorImpl();

  void configFactoryDefault();

  void configBrakeMode(boolean enable);

  void configOutputInverted(boolean invert);

  void configOpenLoopRampRate(double seconds0To100);

  void configClosedLoopRampRate(double seconds0To100);

  void configPID(int slot, double proportional, double integral, double derivative,
                 double velocityFF, double staticFF, double integralZone);

  void setPIDSlot(int slot);

  double[] getPIDConstants(int slot);

  void configCurrentLimit(int maxSupplyCurrent);

  boolean isOutputInverted();

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

  MotorFaults getFaults();

  String getFirmwareVersion();

  double getMaxVelocity();

  double getStallTorque();
}
