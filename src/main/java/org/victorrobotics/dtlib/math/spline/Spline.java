package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.BiFunction;

public abstract class Spline<T extends SplineSegment> implements Iterable<T> {
  protected final List<T> segments;

  protected Spline() {
    segments = new ArrayList<>();
  }

  protected Spline(T segment) {
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

  protected Vector2D_R get(double u, BiFunction<T, Double, Vector2D_R> func) {
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

  public Vector2D_R getPosition(double u) {
    return get(u, SplineSegment::getPosition);
  }

  public Vector2D_R getVelocity(double u) {
    return get(u, SplineSegment::getVelocity);
  }

  public Vector2D_R getAcceleration(double u) {
    return get(u, SplineSegment::getAcceleration);
  }

  public Vector2D_R getJolt(double u) {
    return get(u, SplineSegment::getJolt);
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
