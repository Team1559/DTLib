package org.victorrobotics.dtlib.hardware;

public interface DTAbsoluteEncoderFaults {
  boolean lowVoltage();

  boolean hardwareFailure();

  boolean hasAnyFault();

  boolean other();
}
