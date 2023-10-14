package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

public class CubicBezierControl extends SplineControl {
  private final Vector2D_R p0;
  private final Vector2D_R p1;

  public CubicBezierControl() {
    p0 = new Vector2D_R();
    p1 = new Vector2D_R();
  }

  private CubicBezierControl(Vector2D_R p0, Vector2D_R p1) {
    this.p0 = p0.clone();
    this.p1 = p1.clone();
  }

  public Vector2D_R getP0() {
    return p0.clone();
  }

  public Vector2D_R getP1() {
    return p1.clone();
  }

  public Vector2D_R getP2() {
    return p0.clone()
             .multiply(2)
             .subtract(p1);
  }

  public Vector2D_R getP3() {
    return p0.clone();
  }

  protected Vector2D_R getP0Raw() {
    return p0;
  }

  protected Vector2D_R getP1Raw() {
    return p1;
  }

  protected Vector2D_R getP3Raw() {
    return p0;
  }

  public void setP0(Vector2D_R p0) {
    this.p0.set(p0);
    modCount++;
  }

  public void setP1(Vector2D_R p1) {
    this.p1.set(p1);
    modCount++;
  }

  public void setP2(Vector2D_R p2) {
    this.p1.set(p0.clone()
                  .multiply(2)
                  .subtract(p2));
    modCount++;
  }

  public void setP3(Vector2D_R p3) {
    this.p0.set(p3);
    modCount++;
  }

  public static CubicBezierControl createStart(Vector2D_R p0, Vector2D_R p1) {
    return new CubicBezierControl(p0, p1);
  }

  public static CubicBezierControl createEnd(Vector2D_R p2, Vector2D_R p3) {
    return new CubicBezierControl(p3.clone(), p3.clone()
                                                  .multiply(2)
                                                  .subtract(p2));
  }
}
