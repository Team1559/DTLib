package org.victorrobotics.frc.dtlib.sensor.encoder;

public interface DTAbsoluteEncoderFaults {
  boolean lowVoltage();

  boolean hardwareFailure();

  boolean hasAnyFault();

  boolean other();
}
