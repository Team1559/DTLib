package org.victorrobotics.dtlib.function.supplier;

import java.util.function.IntSupplier;

import edu.wpi.first.util.WPIUtilJNI;

public class DTLimitedIntSupplier implements DTConditionalIntSupplier {
  private final IntSupplier supplier;
  private final long        delayMicros;
  private int               value;
  private long              nextUpdateMicros;

  public DTLimitedIntSupplier(IntSupplier supplier, double rateHz) {
    this.supplier = supplier;
    delayMicros = Math.round(1e6 / rateHz);
    value = supplier.getAsInt();
    nextUpdateMicros = WPIUtilJNI.now() + delayMicros;
  }

  @Override
  public int getAsInt() {
    return value;
  }

  @Override
  public boolean update() {
    long currentTime = WPIUtilJNI.now();
    if (currentTime < nextUpdateMicros) {
      return false;
    }

    int newValue = supplier.getAsInt();
    if (value != newValue) {
      value = newValue;
      nextUpdateMicros = currentTime + delayMicros;
      return true;
    }

    return false;
  }
}
