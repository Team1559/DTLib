package org.victorrobotics.dtlib.function.supplier;

import java.util.function.BooleanSupplier;

import edu.wpi.first.util.WPIUtilJNI;

public class DTLimitedBooleanSupplier implements DTConditionalBooleanSupplier {
  private final BooleanSupplier supplier;
  private final long            delayMicros;
  private boolean               value;
  private long                  nextUpdateMicros;

  public DTLimitedBooleanSupplier(BooleanSupplier supplier, double rateHz) {
    this.supplier = supplier;
    delayMicros = Math.round(1e6 / rateHz);
    value = supplier.getAsBoolean();
    nextUpdateMicros = WPIUtilJNI.now() + delayMicros;
  }

  @Override
  public boolean getAsBoolean() {
    return value;
  }

  @Override
  public boolean update() {
    long currentTime = WPIUtilJNI.now();
    if (currentTime < nextUpdateMicros) {
      return false;
    }

    boolean newValue = supplier.getAsBoolean();
    if (value != newValue) {
      value = newValue;
      nextUpdateMicros = currentTime + delayMicros;
      return true;
    }

    return false;
  }
}
