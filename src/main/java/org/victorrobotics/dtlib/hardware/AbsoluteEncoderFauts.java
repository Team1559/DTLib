package org.victorrobotics.dtlib.hardware;

public interface AbsoluteEncoderFaults {
  boolean lowVoltage();

  boolean hardwareFailure();

  boolean hasAnyFault();

  boolean other();
}
