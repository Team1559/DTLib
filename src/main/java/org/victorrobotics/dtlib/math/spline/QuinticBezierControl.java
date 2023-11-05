package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

public class QuinticBezierControl extends SplineControl {
  private final Vector2D_R p0;
  private final Vector2D_R p1;
  private final Vector2D_R p2;

  public QuinticBezierControl() {
    p0 = new Vector2D_R();
    p1 = new Vector2D_R();
    p2 = new Vector2D_R();
  }

  private QuinticBezierControl(Vector2D_R p0, Vector2D_R p1, Vector2D_R p2) {
    this.p0 = p0;
    this.p1 = p1;
    this.p2 = p2;
  }

  public Vector2D_R getP0() {
    return p0.clone();
  }

  public Vector2D_R getP1() {
    return p1.clone();
  }

  public Vector2D_R getP2() {
    return p2.clone();
  }

  public Vector2D_R getP3() {
    return p0.clone()
             .multiply(2)
             .subtract(p2);
  }

  public Vector2D_R getP4() {
    return p0.clone()
             .multiply(2)
             .subtract(p1);
  }

  public Vector2D_R getP5() {
    return p0.clone();
  }

  protected Vector2D_R getP0Raw() {
    return p0;
  }

  protected Vector2D_R getP1Raw() {
    return p1;
  }

  protected Vector2D_R getP2Raw() {
    return p2;
  }

  protected Vector2D_R getP5Raw() {
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
    this.p2.set(p2);
    modCount++;
  }

  public void setP3(Vector2D_R p3) {
    this.p2.set(p0.clone()
                  .multiply(2)
                  .subtract(p3));
    modCount++;
  }

  public void setP4(Vector2D_R p4) {
    this.p1.set(p0.clone()
                  .multiply(2)
                  .subtract(p4));
    modCount++;
  }

  public void setP5(Vector2D_R p5) {
    this.p0.set(p5);
    modCount++;
  }

  public static QuinticBezierControl createStart(Vector2D_R p0, Vector2D_R p1, Vector2D_R p2) {
    return new QuinticBezierControl(p0.clone(), p1.clone(), p2.clone());
  }

  public static QuinticBezierControl createEnd(Vector2D_R p3, Vector2D_R p4, Vector2D_R p5) {
    return new QuinticBezierControl(p5.clone(), p5.clone()
                                                    .multiply(2)
                                                    .subtract(p4),
                                      p5.clone()
                                        .multiply(2)
                                        .subtract(p3));
  }
}
