package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

public class LinearSpline extends Spline<LinearInterpolationSegment> {
  public LinearSpline() {
    super();
  }

  public LinearSpline(Vector2D_R p0, Vector2D_R p1) {
    super(new LinearInterpolationSegment(p0, p1));
  }

  public LinearSpline(LinearInterpolationSegment segment) {
    super(segment);
  }

  public LinearInterpolationSegment appendSegment(Vector2D_R p1) {
    LinearInterpolationControl prevControl;
    if (segments.isEmpty()) {
      prevControl = new LinearInterpolationControl();
    } else {
      LinearInterpolationSegment prevCurve = segments.get(segments.size() - 1);
      prevControl = prevCurve.getEndControl();
    }

    LinearInterpolationControl endControl = new LinearInterpolationControl(p1);
    LinearInterpolationSegment newSegment = new LinearInterpolationSegment(prevControl, endControl);
    segments.add(newSegment);
    return newSegment;
  }

  public LinearInterpolationSegment prependSegment(Vector2D_R p0) {
    LinearInterpolationControl nextControl;
    if (segments.isEmpty()) {
      nextControl = new LinearInterpolationControl();
    } else {
      LinearInterpolationSegment nextCurve = segments.get(0);
      nextControl = nextCurve.getEndControl();
    }

    LinearInterpolationControl startControl = new LinearInterpolationControl(p0);
    LinearInterpolationSegment newSegment =
        new LinearInterpolationSegment(startControl, nextControl);
    segments.add(0, newSegment);
    return newSegment;
  }

  @Override
  public LinearInterpolationSegment splitSegment(int index, double t) {
    LinearInterpolationSegment toSplit = segments.get(index);
    Vector2D_R pos = toSplit.getPosition(t);
    LinearInterpolationControl splitControl = new LinearInterpolationControl(pos);

    LinearInterpolationSegment before =
        new LinearInterpolationSegment(toSplit.getStartControl(), splitControl);
    LinearInterpolationSegment after =
        new LinearInterpolationSegment(splitControl, toSplit.getEndControl());
    segments.set(index, before);
    segments.add(index + 1, after);

    return toSplit;
  }
}
