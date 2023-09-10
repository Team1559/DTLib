package org.victorrobotics.dtlib;

public interface DTHardwareComponent extends AutoCloseable {
  void initializeHardware();

  @Override
  void close();
}
