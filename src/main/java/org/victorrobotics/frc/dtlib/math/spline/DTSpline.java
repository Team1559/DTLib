package org.victorrobotics.frc.dtlib.math.spline;

import org.victorrobotics.frc.dtlib.math.geometry.DTVector2DR;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.BiFunction;

public abstract class DTSpline<T extends DTCurve> implements Iterable<T> {
    protected final List<T> segments;

    protected DTSpline() {
        segments = new ArrayList<>();
    }

    protected DTSpline(T segment) {
        segments = new ArrayList<>();
        segments.add(segment);
    }

    public abstract T splitSegment(int index, double t);

    public T split(double u) {
        if (segments.isEmpty() || !Double.isFinite(u) || u < 0 || u >= segments.size()) {
            return null;
        }

        int index = (int) u;
        return splitSegment(index, u % 1);
    }

    protected DTVector2DR get(double u, BiFunction<T, Double, DTVector2DR> func) {
        if (segments.isEmpty() || !Double.isFinite(u)) {
            return null;
        } else if (u < 0) {
            return func.apply(segments.get(0), 0D);
        } else if (u >= segments.size()) {
            return func.apply(segments.get(segments.size() - 1), 1D);
        }

        int index = (int) u;
        u %= 1;
        return func.apply(segments.get(index), u);
    }

    public DTVector2DR getPosition(double u) {
        return get(u, DTCurve::getPosition);
    }

    public DTVector2DR getVelocity(double u) {
        return get(u, DTCurve::getVelocity);
    }

    public DTVector2DR getAcceleration(double u) {
        return get(u, DTCurve::getAcceleration);
    }

    public DTVector2DR getJolt(double u) {
        return get(u, DTCurve::getJolt);
    }

    public List<T> getSegments() {
        return List.copyOf(segments);
    }

    public int length() {
        return segments.size();
    }

    public T getSegment(int index) {
        return segments.get(index);
    }

    public T removeFirstSegment() {
        return segments.isEmpty() ? null : segments.remove(0);
    }

    public T removeLastSegment() {
        return segments.isEmpty() ? null : segments.remove(segments.size() - 1);
    }

    public void clear() {
        segments.clear();
    }

    @Override
    public Iterator<T> iterator() {
        return segments.iterator();
    }
}
