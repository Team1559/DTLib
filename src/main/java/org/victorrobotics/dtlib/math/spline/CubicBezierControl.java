package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.DTVector2dR;

public class DTCubicBezierControl extends DTCurveControl {
  private final DTVector2dR p0;
  private final DTVector2dR p1;

  public DTCubicBezierControl() {
    p0 = new DTVector2dR();
    p1 = new DTVector2dR();
  }

  private DTCubicBezierControl(DTVector2dR p0, DTVector2dR p1) {
    this.p0 = p0.clone();
    this.p1 = p1.clone();
  }

  public DTVector2dR getP0() {
    return p0.clone();
  }

  public DTVector2dR getP1() {
    return p1.clone();
  }

  public DTVector2dR getP2() {
    return p0.clone()
             .multiply(2)
             .subtract(p1);
  }

  public DTVector2dR getP3() {
    return p0.clone();
  }

  protected DTVector2dR getP0Raw() {
    return p0;
  }

  protected DTVector2dR getP1Raw() {
    return p1;
  }

  protected DTVector2dR getP3Raw() {
    return p0;
  }

  public void setP0(DTVector2dR p0) {
    this.p0.set(p0);
    modCount++;
  }

  public void setP1(DTVector2dR p1) {
    this.p1.set(p1);
    modCount++;
  }

  public void setP2(DTVector2dR p2) {
    this.p1.set(p0.clone()
                  .multiply(2)
                  .subtract(p2));
    modCount++;
  }

  public void setP3(DTVector2dR p3) {
    this.p0.set(p3);
    modCount++;
  }

  public static DTCubicBezierControl createStart(DTVector2dR p0, DTVector2dR p1) {
    return new DTCubicBezierControl(p0, p1);
  }

  public static DTCubicBezierControl createEnd(DTVector2dR p2, DTVector2dR p3) {
    return new DTCubicBezierControl(p3.clone(), p3.clone()
                                                  .multiply(2)
                                                  .subtract(p2));
  }
}
