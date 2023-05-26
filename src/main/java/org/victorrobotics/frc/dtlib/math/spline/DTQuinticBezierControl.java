package org.victorrobotics.frc.dtlib.math.spline;

import org.victorrobotics.frc.dtlib.math.geometry.DTVector2DR;

public class DTQuinticBezierControl implements DTCurveControl {
    private final DTVector2DR p0;
    private final DTVector2DR p1;
    private final DTVector2DR p2;
    private int               modCount;

    public DTQuinticBezierControl() {
        p0 = new DTVector2DR();
        p1 = new DTVector2DR();
        p2 = new DTVector2DR();
    }

    private DTQuinticBezierControl(DTVector2DR p0, DTVector2DR p1, DTVector2DR p2) {
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
    }

    @Override
    public int getModCount() {
        return modCount;
    }

    public DTVector2DR getP0() {
        return p0.clone();
    }

    public DTVector2DR getP1() {
        return p1.clone();
    }

    public DTVector2DR getP2() {
        return p2.clone();
    }

    public DTVector2DR getP3() {
        return p0.clone()
                 .multiply(2)
                 .subtract(p2);
    }

    public DTVector2DR getP4() {
        return p0.clone()
                 .multiply(2)
                 .subtract(p1);
    }

    public DTVector2DR getP5() {
        return p0.clone();
    }

    protected DTVector2DR getP0Raw() {
        return p0;
    }

    protected DTVector2DR getP1Raw() {
        return p1;
    }

    protected DTVector2DR getP2Raw() {
        return p2;
    }

    protected DTVector2DR getP5Raw() {
        return p0;
    }

    public void setP0(DTVector2DR p0) {
        this.p0.set(p0);
        modCount++;
    }

    public void setP1(DTVector2DR p1) {
        this.p1.set(p1);
        modCount++;
    }

    public void setP2(DTVector2DR p2) {
        this.p2.set(p2);
        modCount++;
    }

    public void setP3(DTVector2DR p3) {
        this.p2.set(p0.clone()
                      .multiply(2)
                      .subtract(p3));
        modCount++;
    }

    public void setP4(DTVector2DR p4) {
        this.p1.set(p0.clone()
                      .multiply(2)
                      .subtract(p4));
        modCount++;
    }

    public void setP5(DTVector2DR p5) {
        this.p0.set(p5);
        modCount++;
    }

    public static DTQuinticBezierControl createStart(DTVector2DR p0, DTVector2DR p1, DTVector2DR p2) {
        return new DTQuinticBezierControl(p0.clone(), p1.clone(), p2.clone());
    }

    public static DTQuinticBezierControl createEnd(DTVector2DR p3, DTVector2DR p4, DTVector2DR p5) {
        return new DTQuinticBezierControl(p5.clone(), p5.clone()
                                                        .multiply(2)
                                                        .subtract(p4),
                p5.clone()
                  .multiply(2)
                  .subtract(p3));
    }
}
