package org.victorrobotics.frc.dtlib.math.spline;

import org.victorrobotics.frc.dtlib.math.geometry.DTVector2DR;

public class DTCubicBezierSpline extends DTSpline<DTCubicBezierCurve> {
    public DTCubicBezierSpline() {
        super();
    }

    public DTCubicBezierSpline(DTVector2DR p0, DTVector2DR p1, DTVector2DR p2, DTVector2DR p3) {
        super(new DTCubicBezierCurve(p0, p1, p2, p3));
    }

    public DTCubicBezierSpline(DTCubicBezierCurve segment) {
        super(segment);
    }

    public DTCubicBezierCurve appendSegment(DTVector2DR p2, DTVector2DR p3) {
        DTCubicBezierControl prevControl;
        if (segments.isEmpty()) {
            prevControl = new DTCubicBezierControl();
        } else {
            DTCubicBezierCurve prevCurve = segments.get(segments.size() - 1);
            prevControl = prevCurve.getEndControl();
        }

        DTCubicBezierControl endControl = DTCubicBezierControl.createEnd(p2, p3);
        DTCubicBezierCurve newSegment = new DTCubicBezierCurve(prevControl, endControl);
        segments.add(newSegment);
        return newSegment;
    }

    public DTCubicBezierCurve prependSegment(DTVector2DR p0, DTVector2DR p1) {
        DTCubicBezierControl nextControl;
        if (segments.isEmpty()) {
            nextControl = new DTCubicBezierControl();
        } else {
            DTCubicBezierCurve nextCurve = segments.get(0);
            nextControl = nextCurve.getEndControl();
        }

        DTCubicBezierControl startControl = DTCubicBezierControl.createStart(p0, p1);
        DTCubicBezierCurve newSegment = new DTCubicBezierCurve(startControl, nextControl);
        segments.add(0, newSegment);
        return newSegment;
    }

    @Override
    public DTCubicBezierCurve splitSegment(int index, double t) {
        DTCubicBezierCurve toSplit = segments.get(index);
        DTVector2DR p0 = toSplit.getPosition(t);
        DTVector2DR p1 = toSplit.getVelocity(t)
                                .multiply(1 / 3D);
        DTCubicBezierControl splitControl = DTCubicBezierControl.createStart(p0, p1);

        DTCubicBezierCurve before = new DTCubicBezierCurve(toSplit.getStartControl(), splitControl);
        DTCubicBezierCurve after = new DTCubicBezierCurve(splitControl, toSplit.getEndControl());
        segments.set(index, before);
        segments.add(index + 1, after);

        return toSplit;
    }
}
