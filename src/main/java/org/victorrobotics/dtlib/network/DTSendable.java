package org.victorrobotics.dtlib.network;

import org.victorrobotics.dtlib.function.supplier.DTLimitedBooleanSupplier;
import org.victorrobotics.dtlib.function.supplier.DTLimitedDoubleSupplier;
import org.victorrobotics.dtlib.function.supplier.DTLimitedFloatSupplier;
import org.victorrobotics.dtlib.function.supplier.DTLimitedLongSupplier;
import org.victorrobotics.dtlib.function.supplier.DTLimitedSupplier;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public interface DTSendable extends Sendable, AutoCloseable {
  double UPDATE_RATE_FAST_HZ = 25;
  double UPDATE_RATE_STD_HZ  = 10;
  double UPDATE_RATE_SLOW_HZ = 2;

  @Override
  void initSendable(SendableBuilder builder);

  @Override
  void close() throws Exception;

  default DTLimitedDoubleSupplier limitRate(DoubleSupplier supplier, double rateHz) {
    return new DTLimitedDoubleSupplier(supplier, rateHz);
  }

  default DTLimitedFloatSupplier limitRate(FloatSupplier supplier, double rateHz) {
    return new DTLimitedFloatSupplier(supplier, rateHz);
  }

  default DTLimitedBooleanSupplier limitRate(BooleanSupplier supplier, double rateHz) {
    return new DTLimitedBooleanSupplier(supplier, rateHz);
  }

  default DTLimitedLongSupplier limitRate(LongSupplier supplier, double rateHz) {
    return new DTLimitedLongSupplier(supplier, rateHz);
  }

  default <T> DTLimitedSupplier<T> limitRate(Supplier<T> supplier, double rateHz) {
    return new DTLimitedSupplier<>(supplier, rateHz);
  }
}
