package org.victorrobotics.frc.dtlib;

public interface DTHardwareComponent extends AutoCloseable {
  void initializeHardware();

  @Override
  void close();
}
