package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

/**
 * A third-order Bézier spline, consisting of cubic bezier segments with
 * continuous velocity (C1 continuity). At each join, the two segments will have
 * equal positions and velocities.
 *
 * @see Spline
 * @see <a href=
 *      "https://en.wikipedia.org/wiki/Bézier_curve#Cubic_Bézier_curves">Wikipedia</a>
 */
public class CubicBezierSpline extends Spline<CubicBezierSegment> {
  /**
   * Constructs a CubicBezierSpline with no curve segments.
   */
  public CubicBezierSpline() {
    super();
  }

  /**
   * Constructs a CubicBezierSpline with the given points creating the first
   * curve segment.
   *
   * @param p0
   *        Bézier control point 0
   * @param p1
   *        Bézier control point 1
   * @param p2
   *        Bézier control point 2
   * @param p3
   *        Bézier control point 3
   */
  public CubicBezierSpline(Vector2D_R p0, Vector2D_R p1, Vector2D_R p2, Vector2D_R p3) {
    super(new CubicBezierSegment(p0, p1, p2, p3));
  }

  /**
   * Constructs a CubicBezierSpline with the given curve segment.
   *
   * @param segment
   *        the cubic Bézier segment
   */
  public CubicBezierSpline(CubicBezierSegment segment) {
    super(segment);
  }

  /**
   * Appends a new curve segment onto the end of this spline. If this spline has
   * no existing segments, the new segment will begin with a default point.
   * Bézier control points 0 and 1 are inherited from the previous segment.
   *
   * @param p2
   *        Bézier control point 2
   * @param p3
   *        Bézier control point 3
   *
   * @return the newly appended curve segment
   */
  public CubicBezierSegment appendSegment(Vector2D_R p2, Vector2D_R p3) {
    CubicBezierControl prevControl;
    if (segments.isEmpty()) {
      prevControl = new CubicBezierControl();
    } else {
      CubicBezierSegment prevCurve = segments.get(segments.size() - 1);
      prevControl = prevCurve.getEndControl();
    }

    CubicBezierControl endControl = CubicBezierControl.createEnd(p2, p3);
    CubicBezierSegment newSegment = new CubicBezierSegment(prevControl, endControl);
    segments.add(newSegment);
    return newSegment;
  }

  /**
   * Prepends a new curve segment onto the beginning of this spline. If this
   * spline has no existing segments, the new segment will end with a default
   * point. Bézier control points 2 and 3 are inherited from the next segment.
   *
   * @param p0
   *        Bézier control point 0
   * @param p1
   *        Bézier control point 1
   *
   * @return the newly prepended curve segment
   */
  public CubicBezierSegment prependSegment(Vector2D_R p0, Vector2D_R p1) {
    CubicBezierControl nextControl;
    if (segments.isEmpty()) {
      nextControl = new CubicBezierControl();
    } else {
      CubicBezierSegment nextCurve = segments.get(0);
      nextControl = nextCurve.getEndControl();
    }

    CubicBezierControl startControl = CubicBezierControl.createStart(p0, p1);
    CubicBezierSegment newSegment = new CubicBezierSegment(startControl, nextControl);
    segments.add(0, newSegment);
    return newSegment;
  }

  @Override
  public CubicBezierSegment splitSegment(int index, double t) {
    CubicBezierSegment toSplit = segments.get(index);
    Vector2D_R p0 = toSplit.getPosition(t);
    Vector2D_R p1 = toSplit.getVelocity(t)
                            .multiply(1 / 3D);
    CubicBezierControl splitControl = CubicBezierControl.createStart(p0, p1);

    CubicBezierSegment before = new CubicBezierSegment(toSplit.getStartControl(), splitControl);
    CubicBezierSegment after = new CubicBezierSegment(splitControl, toSplit.getEndControl());
    segments.set(index, before);
    segments.add(index + 1, after);

    return toSplit;
  }
}
