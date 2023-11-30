package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.BiFunction;

/**
 * A superclass for splines that consist of one or more curve segments of a
 * certain type.
 */
public abstract class Spline<T extends SplineSegment> implements Iterable<T> {
  /**
   * The ordered list of connected curve segments that make up this spline
   */
  protected final List<T> segments;

  /**
   * Constructs a Spline with no curve segments.
   */
  protected Spline() {
    segments = new ArrayList<>();
  }

  /**
   * Constructs a Spline with the given curve segment.
   *
   * @param segment the segment to include
   */
  protected Spline(T segment) {
    segments = new ArrayList<>();
    segments.add(segment);
  }

  /**
   * Splits the specified curve segment into two segments at the specified
   * point. The resulting path should not change, only adding a control point
   * somewhere in the middle.
   *
   * @param index the index of the curve segment to split
   * @param t the "time" on that segment at which to split in the range (0, 1)
   * @return the newly created curve control at the split location
   */
  public abstract SplineControl splitSegment(int index, double t);

  /**
   * Splits the specified curve segment into two segments at the specified
   * point. The resulting path should not change, only adding a control point
   * somewhere in the middle.
   *
   * @param u the "time" at which to split
   * @return the newly created curve control at the split location
   */
  public SplineControl split(double u) {
    if (segments.isEmpty() || !Double.isFinite(u) || u < 0 || u >= segments.size()) {
      return null;
    }

    int index = (int) u;
    return splitSegment(index, u % 1);
  }

  private Vector2D_R get(double u, BiFunction<T, Double, Vector2D_R> func) {
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

  /**
   * Gets the position of this spline at the specified point.
   *
   * @param u the desired "time"
   * @return a vector position on this spline
   */
  public Vector2D_R getPosition(double u) {
    return get(u, SplineSegment::getPosition);
  }

  /**
   * Gets the velocity of this spline at the specified point.
   *
   * @param u the desired "time"
   * @return a vector velocity on this spline
   */
  public Vector2D_R getVelocity(double u) {
    return get(u, SplineSegment::getVelocity);
  }

  /**
   * Gets the acceleration of this spline at the specified point.
   *
   * @param u the desired "time"
   * @return a vector acceleration on this spline
   */
  public Vector2D_R getAcceleration(double u) {
    return get(u, SplineSegment::getAcceleration);
  }

  /**
   * Gets the jolt of this spline at the specified point.
   *
   * @param u the desired "time"
   * @return a vector jolt on this spline
   */
  public Vector2D_R getJolt(double u) {
    return get(u, SplineSegment::getJolt);
  }

  /**
   * @return a copy of the backing list of curve segments.
   */
  public List<T> getSegments() {
    return List.copyOf(segments);
  }

  /**
   * @return the number of curve segments in this spline
   */
  public int length() {
    return segments.size();
  }

  /**
   * Gets the curve segment at the specified index.
   *
   * @param index the desired index
   * @return the segment at that index
   */
  public T getSegment(int index) {
    return segments.get(index);
  }

  /**
   * Removes the curve segment at the beginning of this spline, without
   * impacting subsequent segments.
   *
   * @return the removed curve segment
   */
  public T removeFirstSegment() {
    return segments.isEmpty() ? null : segments.remove(0);
  }

  /**
   * Removes the curve segment at the end of this spline, without impacting
   * subsequent segments.
   *
   * @return the removed curve segment
   */
  public T removeLastSegment() {
    return segments.isEmpty() ? null : segments.remove(segments.size() - 1);
  }

  /**
   * Removes all curve segments from this spline, rendering it empty.
   */
  public void clear() {
    segments.clear();
  }

  @Override
  public Iterator<T> iterator() {
    return segments.iterator();
  }
}
