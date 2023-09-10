package org.victorrobotics.dtlib.function.supplier;

import java.util.function.LongSupplier;

import edu.wpi.first.util.WPIUtilJNI;

public class DTLimitedLongSupplier implements DTConditionalLongSupplier {
  private final LongSupplier supplier;
  private final long         delayMicros;
  private long               value;
  private long               nextUpdateMicros;

  public DTLimitedLongSupplier(LongSupplier supplier, double rateHz) {
    this.supplier = supplier;
    delayMicros = Math.round(1e6 / rateHz);
    value = supplier.getAsLong();
    nextUpdateMicros = WPIUtilJNI.now() + delayMicros;
  }

  @Override
  public long getAsLong() {
    return value;
  }

  @Override
  public boolean update() {
    long currentTime = WPIUtilJNI.now();
    if (currentTime < nextUpdateMicros) {
      return false;
    }

    long newValue = supplier.getAsLong();
    if (value != newValue) {
      value = newValue;
      nextUpdateMicros = currentTime + delayMicros;
      return true;
    }

    return false;
  }
}
