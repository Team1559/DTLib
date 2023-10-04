package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.DTVector2dR;

public class DTQuinticBezierControl extends DTCurveControl {
  private final DTVector2dR p0;
  private final DTVector2dR p1;
  private final DTVector2dR p2;

  public DTQuinticBezierControl() {
    p0 = new DTVector2dR();
    p1 = new DTVector2dR();
    p2 = new DTVector2dR();
  }

  private DTQuinticBezierControl(DTVector2dR p0, DTVector2dR p1, DTVector2dR p2) {
    this.p0 = p0;
    this.p1 = p1;
    this.p2 = p2;
  }

  public DTVector2dR getP0() {
    return p0.clone();
  }

  public DTVector2dR getP1() {
    return p1.clone();
  }

  public DTVector2dR getP2() {
    return p2.clone();
  }

  public DTVector2dR getP3() {
    return p0.clone()
             .multiply(2)
             .subtract(p2);
  }

  public DTVector2dR getP4() {
    return p0.clone()
             .multiply(2)
             .subtract(p1);
  }

  public DTVector2dR getP5() {
    return p0.clone();
  }

  protected DTVector2dR getP0Raw() {
    return p0;
  }

  protected DTVector2dR getP1Raw() {
    return p1;
  }

  protected DTVector2dR getP2Raw() {
    return p2;
  }

  protected DTVector2dR getP5Raw() {
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
    this.p2.set(p2);
    modCount++;
  }

  public void setP3(DTVector2dR p3) {
    this.p2.set(p0.clone()
                  .multiply(2)
                  .subtract(p3));
    modCount++;
  }

  public void setP4(DTVector2dR p4) {
    this.p1.set(p0.clone()
                  .multiply(2)
                  .subtract(p4));
    modCount++;
  }

  public void setP5(DTVector2dR p5) {
    this.p0.set(p5);
    modCount++;
  }

  public static DTQuinticBezierControl createStart(DTVector2dR p0, DTVector2dR p1, DTVector2dR p2) {
    return new DTQuinticBezierControl(p0.clone(), p1.clone(), p2.clone());
  }

  public static DTQuinticBezierControl createEnd(DTVector2dR p3, DTVector2dR p4, DTVector2dR p5) {
    return new DTQuinticBezierControl(p5.clone(), p5.clone()
                                                    .multiply(2)
                                                    .subtract(p4),
        p5.clone()
          .multiply(2)
          .subtract(p3));
  }
}
