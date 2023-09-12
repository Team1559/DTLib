package org.victorrobotics.dtlib.function.supplier;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.WPIUtilJNI;

public class DTLimitedDoubleSupplier implements DTConditionalDoubleSupplier {
  private final DoubleSupplier supplier;
  private final long           delayMicros;
  private double               value;
  private long                 nextUpdateMicros;

  public DTLimitedDoubleSupplier(DoubleSupplier supplier, double rateHz) {
    this.supplier = supplier;
    delayMicros = Math.round(1e6 / rateHz);
    value = supplier.getAsDouble();
    nextUpdateMicros = WPIUtilJNI.now() + delayMicros;
  }

  @Override
  public double getAsDouble() {
    return value;
  }

  @Override
  public boolean update() {
    long currentTime = WPIUtilJNI.now();
    if (currentTime < nextUpdateMicros) {
      return false;
    }

    double newValue = supplier.getAsDouble();
    if (value != newValue) {
      value = newValue;
      nextUpdateMicros = currentTime + delayMicros;
      return true;
    }

    return false;
  }
}
