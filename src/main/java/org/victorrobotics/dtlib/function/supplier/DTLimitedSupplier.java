package org.victorrobotics.dtlib.function.supplier;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.util.WPIUtilJNI;

public class DTLimitedSupplier<T> implements DTConditionalSupplier<T> {
  private final Supplier<T> supplier;
  private final long        delayMicros;
  private T                 value;
  private long              nextUpdateMicros;

  public DTLimitedSupplier(Supplier<T> supplier, double rateHz) {
    this.supplier = supplier;
    delayMicros = Math.round(1e6 / rateHz);
    value = supplier.get();
    nextUpdateMicros = WPIUtilJNI.now() + delayMicros;
  }

  @Override
  public T get() {
    return value;
  }

  @Override
  public boolean update() {
    long currentTime = WPIUtilJNI.now();
    if (currentTime < nextUpdateMicros) {
      return false;
    }

    T newValue = supplier.get();
    if (!Objects.equals(value, newValue)) {
      value = newValue;
      nextUpdateMicros = currentTime + delayMicros;
      return true;
    }

    return false;
  }
}
