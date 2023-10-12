package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.DTVector2DR;

public class DTLinearSpline extends DTSpline<DTLinearInterpolationCurve> {
  public DTLinearSpline() {
    super();
  }

  public DTLinearSpline(DTVector2DR p0, DTVector2DR p1) {
    super(new DTLinearInterpolationCurve(p0, p1));
  }

  public DTLinearSpline(DTLinearInterpolationCurve segment) {
    super(segment);
  }

  public DTLinearInterpolationCurve appendSegment(DTVector2DR p1) {
    DTLinearInterpolationControl prevControl;
    if (segments.isEmpty()) {
      prevControl = new DTLinearInterpolationControl();
    } else {
      DTLinearInterpolationCurve prevCurve = segments.get(segments.size() - 1);
      prevControl = prevCurve.getEndControl();
    }

    DTLinearInterpolationControl endControl = new DTLinearInterpolationControl(p1);
    DTLinearInterpolationCurve newSegment = new DTLinearInterpolationCurve(prevControl, endControl);
    segments.add(newSegment);
    return newSegment;
  }

  public DTLinearInterpolationCurve prependSegment(DTVector2DR p0) {
    DTLinearInterpolationControl nextControl;
    if (segments.isEmpty()) {
      nextControl = new DTLinearInterpolationControl();
    } else {
      DTLinearInterpolationCurve nextCurve = segments.get(0);
      nextControl = nextCurve.getEndControl();
    }

    DTLinearInterpolationControl startControl = new DTLinearInterpolationControl(p0);
    DTLinearInterpolationCurve newSegment =
        new DTLinearInterpolationCurve(startControl, nextControl);
    segments.add(0, newSegment);
    return newSegment;
  }

  @Override
  public DTLinearInterpolationCurve splitSegment(int index, double t) {
    DTLinearInterpolationCurve toSplit = segments.get(index);
    DTVector2DR pos = toSplit.getPosition(t);
    DTLinearInterpolationControl splitControl = new DTLinearInterpolationControl(pos);

    DTLinearInterpolationCurve before =
        new DTLinearInterpolationCurve(toSplit.getStartControl(), splitControl);
    DTLinearInterpolationCurve after =
        new DTLinearInterpolationCurve(splitControl, toSplit.getEndControl());
    segments.set(index, before);
    segments.add(index + 1, after);

    return toSplit;
  }
}
