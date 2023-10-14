package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

/**
 * A spline made up of continuously interpolated line segments.
 */
public class LinearSpline extends Spline<LinearInterpolationSegment> {
  /**
   * Constructs a DTLinearSpline with no segments.
   */
  public LinearSpline() {
    super();
  }

  /**
   * Constructs a DTLinearSpline starting and ending at the specified points.
   *
   * @param start the start point
   * @param end the end point
   */
  public LinearSpline(Vector2D_R start, Vector2D_R end) {
    super(new LinearInterpolationSegment(start, end));
  }

  /**
   * Constructs a DTLinearSpline consisting of the given line segment.
   *
   * @param segment the original line segment
   */
  public LinearSpline(LinearInterpolationSegment segment) {
    super(segment);
  }

  /**
   * Appends a new line segment onto the end of this spline.
   *
   * @param point the point at which to end the segment
   * @return the new line segment
   */
  public LinearInterpolationSegment appendSegment(Vector2D_R point) {
    LinearInterpolationControl prevControl;
    if (segments.isEmpty()) {
      prevControl = new LinearInterpolationControl();
    } else {
      LinearInterpolationSegment prevCurve = segments.get(segments.size() - 1);
      prevControl = prevCurve.getEndControl();
    }

    LinearInterpolationControl endControl = new LinearInterpolationControl(point);
    LinearInterpolationSegment newSegment = new LinearInterpolationSegment(prevControl, endControl);
    segments.add(newSegment);
    return newSegment;
  }

  /**
   * Prepends a new line segment onto the beginning of this spline.
   *
   * @param point the point at which to start the segment
   * @return the new line segment
   */
  public LinearInterpolationSegment prependSegment(Vector2D_R point) {
    LinearInterpolationControl nextControl;
    if (segments.isEmpty()) {
      nextControl = new LinearInterpolationControl();
    } else {
      LinearInterpolationSegment nextCurve = segments.get(0);
      nextControl = nextCurve.getEndControl();
    }

    LinearInterpolationControl startControl = new LinearInterpolationControl(point);
    LinearInterpolationSegment newSegment =
        new LinearInterpolationSegment(startControl, nextControl);
    segments.add(0, newSegment);
    return newSegment;
  }

  @Override
  public LinearInterpolationControl splitSegment(int index, double t) {
    LinearInterpolationSegment toSplit = segments.get(index);
    Vector2D_R pos = toSplit.getPosition(t);
    LinearInterpolationControl splitControl = new LinearInterpolationControl(pos);

    LinearInterpolationSegment before =
        new LinearInterpolationSegment(toSplit.getStartControl(), splitControl);
    LinearInterpolationSegment after =
        new LinearInterpolationSegment(splitControl, toSplit.getEndControl());
    segments.set(index, before);
    segments.add(index + 1, after);

    return splitControl;
  }
}
