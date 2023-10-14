package org.victorrobotics.dtlib.hardware;

public interface MotorFaults {
  boolean hasAnyFault();

  boolean lowVoltage();

  boolean other();

  boolean softLimitForward();

  boolean softLimitReverse();

  boolean hardLimitForward();

  boolean hardLimitReverse();

  boolean hasReset();

  boolean hardwareFailure();
}
