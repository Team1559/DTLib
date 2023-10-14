package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

public class CubicBezierSpline extends Spline<CubicBezierSegment> {
  public CubicBezierSpline() {
    super();
  }

  public CubicBezierSpline(Vector2D_R p0, Vector2D_R p1, Vector2D_R p2, Vector2D_R p3) {
    super(new CubicBezierSegment(p0, p1, p2, p3));
  }

  public CubicBezierSpline(CubicBezierSegment segment) {
    super(segment);
  }

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
