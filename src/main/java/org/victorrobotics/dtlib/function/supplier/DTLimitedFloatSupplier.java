package org.victorrobotics.dtlib.function.supplier;

import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.function.FloatSupplier;

public class DTLimitedFloatSupplier implements DTConditionalFloatSupplier {
  private final FloatSupplier supplier;
  private final long          delayMicros;
  private float               value;
  private long                nextUpdateMicros;

  public DTLimitedFloatSupplier(FloatSupplier supplier, double rateHz) {
    this.supplier = supplier;
    delayMicros = Math.round(1e6 / rateHz);
    value = supplier.getAsFloat();
    nextUpdateMicros = WPIUtilJNI.now() + delayMicros;
  }

  @Override
  public float getAsFloat() {
    return value;
  }

  @Override
  public boolean update() {
    long currentTime = WPIUtilJNI.now();
    if (currentTime < nextUpdateMicros) {
      return false;
    }

    float newValue = supplier.getAsFloat();
    if (value != newValue) {
      value = newValue;
      nextUpdateMicros = currentTime + delayMicros;
      return true;
    }

    return false;
  }
}
