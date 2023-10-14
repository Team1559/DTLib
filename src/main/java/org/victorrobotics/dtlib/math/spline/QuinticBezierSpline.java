package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

public class QuinticBezierSpline extends Spline<QuinticBezierSegment> {
  public QuinticBezierSpline() {
    super();
  }

  public QuinticBezierSpline(Vector2D_R p0, Vector2D_R v0, Vector2D_R a0, Vector2D_R p1,
                               Vector2D_R v1, Vector2D_R a1) {
    super(new QuinticBezierSegment(p0, v0, a0, p1, v1, a1));
  }

  public QuinticBezierSpline(QuinticBezierSegment segment) {
    super(segment);
  }

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
  public QuinticBezierSegment splitSegment(int index, double t) {
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

    return toSplit;
  }
}
