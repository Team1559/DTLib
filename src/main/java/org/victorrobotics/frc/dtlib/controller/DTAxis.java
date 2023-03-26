package org.victorrobotics.frc.dtlib.controller;

import java.util.function.BiFunction;
import java.util.function.DoubleBinaryOperator;
import java.util.function.DoublePredicate;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

public class DTAxis implements DoubleSupplier {
    private final DoubleSupplier input;

    public DTAxis(DoubleSupplier input) {
        this.input = input;
    }

    @Override
    public double getAsDouble() {
        return input.getAsDouble();
    }

    public DTAxis filter(DoublePredicate filter, double defaultValue) {
        return new DTAxis(() -> {
            double val = input.getAsDouble();
            return filter.test(val) ? val : defaultValue;
        });
    }

    public DTAxis map(DoubleUnaryOperator mapper) {
        return new DTAxis(() -> mapper.applyAsDouble(input.getAsDouble()));
    }

    public DTAxis map(DoubleBinaryOperator mapper, double param2) {
        return new DTAxis(() -> mapper.applyAsDouble(input.getAsDouble(), param2));
    }

    public DTAxis map(DoubleBinaryOperator mapper, DoubleSupplier param2) {
        return new DTAxis(() -> mapper.applyAsDouble(input.getAsDouble(), param2.getAsDouble()));
    }

    public DTAxis map(DoubleBinaryOperator mapper, DoubleUnaryOperator param2) {
        return new DTAxis(() -> {
            double val = input.getAsDouble();
            return mapper.applyAsDouble(val, param2.applyAsDouble(val));
        });
    }

    public <T> DTAxis map(BiFunction<Double, T, Double> mapper, T param2) {
        return new DTAxis(() -> mapper.apply(input.getAsDouble(), param2));
    }

    public <T> DTAxis map(BiFunction<Double, T, Double> mapper, Supplier<T> param2) {
        return new DTAxis(() -> mapper.apply(input.getAsDouble(), param2.get()));
    }

    public DTAxis absolute() {
        return map(Math::abs);
    }

    public DTAxis squareKeepSign() {
        return map((double d1, double d2) -> Math.copySign(d1 * d1, d2), (double d) -> d * d);
    }

    public DTAxis deadband(double minimum, double maximum) {
        requireFinite(minimum);
        requireFinite(maximum);

        double minAbs = Math.abs(minimum);
        double maxAbs = Math.abs(maximum);
        double range = maxAbs - minAbs;

        return map((double d) -> {
            if (Math.abs(d) <= minAbs) {
                return 0;
            } else if (d < 0) {
                return (d + minAbs) / range;
            } else {
                return (d - minAbs) / range;
            }
        });
    }

    public DTTrigger whenGreaterThan(double value) {
        requireFinite(value);
        return new DTTrigger(() -> input.getAsDouble() > value);
    }

    public DTTrigger whenLessThan(double value) {
        requireFinite(value);
        return new DTTrigger(() -> input.getAsDouble() < value);
    }

    public DTTrigger whenGreaterThanOrEqualTo(double value) {
        requireFinite(value);
        return new DTTrigger(() -> input.getAsDouble() >= value);
    }

    public DTTrigger whenLessThanOrEqualTo(double value) {
        requireFinite(value);
        return new DTTrigger(() -> input.getAsDouble() <= value);
    }

    public DTTrigger whenBetween(double minimum, double maximum) {
        return new DTTrigger(() -> {
            double val = input.getAsDouble();
            return val >= minimum && val <= maximum;
        });
    }

    private static void requireFinite(double param) {
        if (!Double.isFinite(param)) {
            throw new IllegalArgumentException("Expected a finite floating-point parameter");
        }
    }
}
