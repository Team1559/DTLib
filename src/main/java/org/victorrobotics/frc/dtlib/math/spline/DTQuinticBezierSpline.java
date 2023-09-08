package org.victorrobotics.frc.dtlib.math.spline;

import org.victorrobotics.frc.dtlib.math.geometry.DTVector2DR;

public class DTQuinticBezierSpline extends DTSpline<DTQuinticBezierCurve> {
  public DTQuinticBezierSpline() {
    super();
  }

  public DTQuinticBezierSpline(DTVector2DR p0, DTVector2DR v0, DTVector2DR a0, DTVector2DR p1, DTVector2DR v1,
      DTVector2DR a1) {
    super(new DTQuinticBezierCurve(p0, v0, a0, p1, v1, a1));
  }

  public DTQuinticBezierSpline(DTQuinticBezierCurve segment) {
    super(segment);
  }

  public DTQuinticBezierCurve appendSegment(DTVector2DR p3, DTVector2DR p4, DTVector2DR p5) {
    DTQuinticBezierControl prevControl;
    if (segments.isEmpty()) {
      prevControl = new DTQuinticBezierControl();
    } else {
      DTQuinticBezierCurve prevCurve = segments.get(segments.size() - 1);
      prevControl = prevCurve.getEndControl();
    }

    DTQuinticBezierControl endControl = DTQuinticBezierControl.createEnd(p3, p4, p5);
    DTQuinticBezierCurve newSegment = new DTQuinticBezierCurve(prevControl, endControl);
    segments.add(newSegment);
    return newSegment;
  }

  public DTQuinticBezierCurve prependSegment(DTVector2DR p0, DTVector2DR p1, DTVector2DR p2) {
    DTQuinticBezierControl nextControl;
    if (segments.isEmpty()) {
      nextControl = new DTQuinticBezierControl();
    } else {
      DTQuinticBezierCurve nextCurve = segments.get(0);
      nextControl = nextCurve.getEndControl();
    }

    DTQuinticBezierControl startControl = DTQuinticBezierControl.createStart(p0, p1, p2);
    DTQuinticBezierCurve newSegment = new DTQuinticBezierCurve(startControl, nextControl);
    segments.add(0, newSegment);
    return newSegment;
  }

  @Override
  public DTQuinticBezierCurve splitSegment(int index, double t) {
    DTQuinticBezierCurve toSplit = segments.get(index);
    DTVector2DR p0 = toSplit.getPosition(t);
    DTVector2DR vel = toSplit.getVelocity(t);
    DTVector2DR acc = toSplit.getAcceleration(t);
    DTVector2DR p1 = p0.clone()
                       .add(vel.clone()
                               .multiply(0.2D));
    DTVector2DR p2 = p0.clone()
                       .add(vel.multiply(0.4D))
                       .add(acc.multiply(0.5D));
    DTQuinticBezierControl splitControl = DTQuinticBezierControl.createStart(p0, p1, p2);

    DTQuinticBezierCurve before = new DTQuinticBezierCurve(toSplit.getStartControl(), splitControl);
    DTQuinticBezierCurve after = new DTQuinticBezierCurve(splitControl, toSplit.getEndControl());
    segments.set(index, before);
    segments.add(index + 1, after);

    return toSplit;
  }
}
