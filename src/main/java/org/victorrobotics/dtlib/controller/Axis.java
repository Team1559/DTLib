package org.victorrobotics.dtlib.controller;

import org.victorrobotics.dtlib.command.CommandScheduler;

import java.util.function.DoublePredicate;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

public class Axis implements DoubleSupplier {
  private final DoubleSupplier supplier;

  private double value;

  public Axis(DoubleSupplier input) {
    this.supplier = input;
    CommandScheduler.bindCallback(this::refresh);
    refresh();
  }

  private void refresh() {
    value = supplier.getAsDouble();
  }

  @Override
  public double getAsDouble() {
    return value;
  }

  public Axis filter(DoublePredicate filter, double defaultValue) {
    return new Axis(() -> filter.test(value) ? value : defaultValue);
  }

  public Axis map(DoubleUnaryOperator mapper) {
    return new Axis(() -> mapper.applyAsDouble(value));
  }

  public Axis negate() {
    return map(d -> -d);
  }

  public Axis absolute() {
    return map(Math::abs);
  }

  public Axis squareKeepSign() {
    return map(d -> Math.copySign(d * d, d));
  }

  public Axis deadband(double min, double max) {
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

  public Trigger whenGreater(double d) {
    requireFinite(d);
    return new Trigger(() -> value > d);
  }

  public Trigger whenGreaterOrEqual(double d) {
    requireFinite(d);
    return new Trigger(() -> value >= d);
  }

  public Trigger whenLess(double d) {
    requireFinite(d);
    return new Trigger(() -> value < d);
  }

  public Trigger whenLessOrEqual(double d) {
    requireFinite(d);
    return new Trigger(() -> value <= d);
  }

  public Trigger whenInRange(double min, double max) {
    requireFinite(min);
    requireFinite(max);
    return new Trigger(() -> value > min && value < max);
  }

  public Trigger whenInRangeOrEqual(double min, double max) {
    requireFinite(min);
    requireFinite(max);
    return new Trigger(() -> value >= min && value <= max);
  }

  private static void requireFinite(double param) {
    if (!Double.isFinite(param)) {
      throw new IllegalArgumentException("expected a finite floating-point parameter");
    }
  }
}
