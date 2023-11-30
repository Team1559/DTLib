package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

/**
 * An object that synchronizes changes made between multiple
 * {@link QuinticBezierSegment} objects, storing its data in two points. Changes
 * made to the preceding segment will be reflected in the one after, and vice
 * versa.
 */
public class QuinticBezierControl extends SplineControl {
  private final Vector2D_R p0;
  private final Vector2D_R p1;
  private final Vector2D_R p2;

  /**
   * Constructs a new QuinticBezierControl with three default points.
   */
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

  /**
   * @return a copy of Bézier control point 0
   */
  public Vector2D_R getP0() {
    return p0.clone();
  }

  /**
   * @return a copy of Bézier control point 1
   */
  public Vector2D_R getP1() {
    return p1.clone();
  }

  /**
   * @return a copy of Bézier control point 2
   */
  public Vector2D_R getP2() {
    return p2.clone();
  }

  /**
   * @return a copy of Bézier control point 3 (computed)
   */
  public Vector2D_R getP3() {
    return p0.clone()
             .multiply(2)
             .subtract(p2);
  }

  /**
   * @return a copy of Bézier control point 4 (computed)
   */
  public Vector2D_R getP4() {
    return p0.clone()
             .multiply(2)
             .subtract(p1);
  }

  /**
   * @return a copy of Bézier control point 5
   */
  public Vector2D_R getP5() {
    return p0.clone();
  }

  /**
   * @return the backing vector for Bézier control point 0
   */
  protected Vector2D_R getP0Raw() {
    return p0;
  }

  /**
   * @return the backing vector for Bézier control point 1
   */
  protected Vector2D_R getP1Raw() {
    return p1;
  }

  /**
   * @return the backing vector for Bézier control point 2
   */
  protected Vector2D_R getP2Raw() {
    return p2;
  }

  /**
   * @return the backing vector for Bézier control point 5
   */
  protected Vector2D_R getP5Raw() {
    return p0;
  }

  /**
   * Sets Bézier control point 0 to the given vector
   *
   * @param p0 the new vector to set
   */
  public void setP0(Vector2D_R p0) {
    this.p0.set(p0);
    modCount++;
  }

  /**
   * Sets Bézier control point 1 to the given vector
   *
   * @param p1 the new vector to set
   */
  public void setP1(Vector2D_R p1) {
    this.p1.set(p1);
    modCount++;
  }

  /**
   * Sets Bézier control point 2 to the given vector
   *
   * @param p2 the new vector to set
   */
  public void setP2(Vector2D_R p2) {
    this.p2.set(p2);
    modCount++;
  }

  /**
   * Sets Bézier control point 3 to the given vector
   *
   * @param p3 the new vector to set
   */
  public void setP3(Vector2D_R p3) {
    this.p2.set(p0.clone()
                  .multiply(2)
                  .subtract(p3));
    modCount++;
  }

  /**
   * Sets Bézier control point 4 to the given vector
   *
   * @param p4 the new vector to set
   */
  public void setP4(Vector2D_R p4) {
    this.p1.set(p0.clone()
                  .multiply(2)
                  .subtract(p4));
    modCount++;
  }

  /**
   * Sets Bézier control point 5 to the given vector
   *
   * @param p5 the new vector to set
   */
  public void setP5(Vector2D_R p5) {
    this.p0.set(p5);
    modCount++;
  }

  /**
   * Constructs a QuinticBezierControl using the three points at the beginning
   * of a curve segment.
   *
   * @param p0 Bézier control point 0
   * @param p1 Bézier control point 1
   * @param p2 Bézier control point 1
   * @return the control points
   */
  public static QuinticBezierControl createStart(Vector2D_R p0, Vector2D_R p1, Vector2D_R p2) {
    return new QuinticBezierControl(p0.clone(), p1.clone(), p2.clone());
  }

  /**
   * Constructs a QuinticBezierControl using the three points at the end of a
   * curve segment.
   *
   * @param p3 Bézier control point 2
   * @param p4 Bézier control point 3
   * @param p5 Bézier control point 3
   * @return the control points
   */
  public static QuinticBezierControl createEnd(Vector2D_R p3, Vector2D_R p4, Vector2D_R p5) {
    return new QuinticBezierControl(p5.clone(), p5.clone()
                                                  .multiply(2)
                                                  .subtract(p4),
                                    p5.clone()
                                      .multiply(2)
                                      .subtract(p3));
  }
}
