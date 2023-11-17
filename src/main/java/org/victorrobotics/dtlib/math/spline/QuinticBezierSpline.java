package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

/**
 * A fifth-order Bézier spline, consisting of quintic bezier segments with
 * continuous acceleration (C2 continuity). At each join, the two segments will
 * have equal positions, velocities, and accelerations.
 *
 * @see Spline
 * @see <a href= "https://en.wikipedia.org/wiki/Bézier_curve">Wikipedia</a>
 */
public class QuinticBezierSpline extends Spline<QuinticBezierSegment> {
  /**
   * Constructs a QuinticBezierSpline with no curve segments.
   */
  public QuinticBezierSpline() {
    super();
  }

  /**
   * Constructs a QuinticBezierSpline with the given points creating the first
   * curve segment.
   *
   * @param p0 Bézier control point 0
   * @param p3 Bézier control point 1
   * @param p2 Bézier control point 2
   * @param p3 Bézier control point 3
   * @param p4 Bézier control point 3
   * @param p5 Bézier control point 3
   */
  public QuinticBezierSpline(Vector2D_R p0, Vector2D_R p1, Vector2D_R p2, Vector2D_R p3,
                             Vector2D_R p4, Vector2D_R p5) {
    super(new QuinticBezierSegment(p0, p1, p2, p3, p4, p5));
  }

  /**
   * Constructs a QuinticBezierSpline with the given curve segment.
   *
   * @param segment the quintic Bézier segment
   */
  public QuinticBezierSpline(QuinticBezierSegment segment) {
    super(segment);
  }

  /**
   * Appends a new curve segment onto the end of this spline. If this spline has
   * no existing segments, the new segment will begin with a default point.
   * Bézier control points 0, 1, and 2 are inherited from the previous segment.
   *
   * @param p3 Bézier control point 3
   * @param p4 Bézier control point 4
   * @param p5 Bézier control point 5
   * @return the newly appended curve segment
   */
  public QuinticBezierSegment appendSegment(Vector2D_R p3, Vector2D_R p4, Vector2D_R p5) {
    QuinticBezierControl prevControl;
    if (segments.isEmpty()) {
      prevControl = new QuinticBezierControl();
    } else {
      QuinticBezierSegment prevCurve = segments.get(segments.size() - 1);
      prevControl = prevCurve.getEndControl();
    }

    QuinticBezierControl endControl = QuinticBezierControl.createEnd(p3, p4, p5);
    QuinticBezierSegment newSegment = new QuinticBezierSegment(prevControl, endControl);
    segments.add(newSegment);
    return newSegment;
  }

  /**
   * Prepends a new curve segment onto the beginning of this spline. If this
   * spline has no existing segments, the new segment will end with a default
   * point. Bézier control points 3, 4, and 5 are inherited from the next segment.
   *
   * @param p0 Bézier control point 0
   * @param p1 Bézier control point 1
   * @param p2 Bézier control point 2
   * @return the newly prepended curve segment
   */
  public QuinticBezierSegment prependSegment(Vector2D_R p0, Vector2D_R p1, Vector2D_R p2) {
    QuinticBezierControl nextControl;
    if (segments.isEmpty()) {
      nextControl = new QuinticBezierControl();
    } else {
      QuinticBezierSegment nextCurve = segments.get(0);
      nextControl = nextCurve.getEndControl();
    }

    QuinticBezierControl startControl = QuinticBezierControl.createStart(p0, p1, p2);
    QuinticBezierSegment newSegment = new QuinticBezierSegment(startControl, nextControl);
    segments.add(0, newSegment);
    return newSegment;
  }

  @Override
  public QuinticBezierControl splitSegment(int index, double t) {
    QuinticBezierSegment toSplit = segments.get(index);
    Vector2D_R p0 = toSplit.getPosition(t);
    Vector2D_R vel = toSplit.getVelocity(t);
    Vector2D_R acc = toSplit.getAcceleration(t);
    Vector2D_R p1 = p0.clone()
                      .add(vel.clone()
                              .multiply(0.2D));
    Vector2D_R p2 = p0.clone()
                      .add(vel.multiply(0.4D))
                      .add(acc.multiply(0.5D));
    QuinticBezierControl splitControl = QuinticBezierControl.createStart(p0, p1, p2);

    QuinticBezierSegment before = new QuinticBezierSegment(toSplit.getStartControl(), splitControl);
    QuinticBezierSegment after = new QuinticBezierSegment(splitControl, toSplit.getEndControl());
    segments.set(index, before);
    segments.add(index + 1, after);

    return splitControl;
  }
}
