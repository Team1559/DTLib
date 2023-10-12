package org.victorrobotics.dtlib.controller;

import org.victorrobotics.dtlib.command.DTCommandScheduler;
import org.victorrobotics.dtlib.exception.DTIllegalArgumentException;

import java.util.function.DoublePredicate;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

public class DTAxis implements DoubleSupplier {
  private final DoubleSupplier supplier;

  private double value;

  public DTAxis(DoubleSupplier input) {
    this.supplier = input;
    DTCommandScheduler.bindCallback(this::refresh);
    refresh();
  }

  private void refresh() {
    value = supplier.getAsDouble();
  }

  @Override
  public double getAsDouble() {
    return value;
  }

  public DTAxis filter(DoublePredicate filter, double defaultValue) {
    return new DTAxis(() -> filter.test(value) ? value : defaultValue);
  }

  public DTAxis map(DoubleUnaryOperator mapper) {
    return new DTAxis(() -> mapper.applyAsDouble(value));
  }

  public DTAxis negate() {
    return map(d -> -d);
  }

  public DTAxis absolute() {
    return map(Math::abs);
  }

  public DTAxis squareKeepSign() {
    return map(d -> Math.copySign(d * d, d));
  }

  public DTAxis deadband(double min, double max) {
    requireFinite(min);
    requireFinite(max);

    double minAbs = Math.abs(min);
    double maxAbs = Math.abs(max);
    double rangeInv = 1 / (maxAbs - minAbs);

    return map((double d) -> {
      if (Math.abs(d) <= minAbs) {
        return 0;
      } else if (d < 0) {
        return (d + minAbs) * rangeInv;
      } else {
        return (d - minAbs) * rangeInv;
      }
    });
  }

  public DTTrigger whenGreater(double d) {
    requireFinite(d);
    return new DTTrigger(() -> value > d);
  }

  public DTTrigger whenGreaterOrEqual(double d) {
    requireFinite(d);
    return new DTTrigger(() -> value >= d);
  }

  public DTTrigger whenLess(double d) {
    requireFinite(d);
    return new DTTrigger(() -> value < d);
  }

  public DTTrigger whenLessOrEqual(double d) {
    requireFinite(d);
    return new DTTrigger(() -> value <= d);
  }

  public DTTrigger whenInRange(double min, double max) {
    requireFinite(min);
    requireFinite(max);
    return new DTTrigger(() -> value > min && value < max);
  }

  public DTTrigger whenInRangeOrEqual(double min, double max) {
    requireFinite(min);
    requireFinite(max);
    return new DTTrigger(() -> value >= min && value <= max);
  }

  private static void requireFinite(double param) {
    if (!Double.isFinite(param)) {
      throw new DTIllegalArgumentException(param, "expected a finite floating-point parameter");
    }
  }
}
