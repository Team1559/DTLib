package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

/**
 * An object that synchronizes changes made between multiple
 * {@link CubicBezierSegment} objects, storing its data in two points. Changes
 * made to the preceding segment will be reflected in the one after, and vice
 * versa.
 */
public class CubicBezierControl extends SplineControl {
  private final Vector2D_R p0;
  private final Vector2D_R p1;

  /**
   * Constructs a new CubicBezierControl with two default points.
   */
  public CubicBezierControl() {
    p0 = new Vector2D_R();
    p1 = new Vector2D_R();
  }

  private CubicBezierControl(Vector2D_R p0, Vector2D_R p1) {
    this.p0 = p0.clone();
    this.p1 = p1.clone();
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
   * @return a copy of Bézier control point 2 (computed)
   */
  public Vector2D_R getP2() {
    return p0.clone()
             .multiply(2)
             .subtract(p1);
  }

  /**
   * @return a copy of Bézier control point 3 (computed)
   */
  public Vector2D_R getP3() {
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
   * @return the backing vector for Bézier control point 3 (same as 0)
   */
  protected Vector2D_R getP3Raw() {
    return p0;
  }

  /**
   * Sets Bézier control point 0 to the given vector
   *
   * @param p0
   *        the new vector to set
   */
  public void setP0(Vector2D_R p0) {
    this.p0.set(p0);
    modCount++;
  }

  /**
   * Sets Bézier control point 1 to the given vector
   *
   * @param p1
   *        the new vector to set
   */
  public void setP1(Vector2D_R p1) {
    this.p1.set(p1);
    modCount++;
  }

  /**
   * Sets Bézier control point 2 to the given vector
   *
   * @param p2
   *        the new vector to set
   */
  public void setP2(Vector2D_R p2) {
    this.p1.set(p0.clone()
                  .multiply(2)
                  .subtract(p2));
    modCount++;
  }

  /**
   * Sets Bézier control point 3 to the given vector
   *
   * @param p3
   *        the new vector to set
   */
  public void setP3(Vector2D_R p3) {
    this.p0.set(p3);
    modCount++;
  }

  /**
   * Constructs a CubicBezierControl using the two points at the beginning of
   * a curve segment.
   *
   * @param p0
   *        Bézier control point 0
   * @param p1
   *        Bézier control point 1
   *
   * @return the control points
   */
  public static CubicBezierControl createStart(Vector2D_R p0, Vector2D_R p1) {
    return new CubicBezierControl(p0, p1);
  }

  /**
   * Constructs a CubicBezierControl using the two points at the end of a
   * curve segment.
   *
   * @param p2
   *        Bézier control point 2
   * @param p3
   *        Bézier control point 3
   *
   * @return the control points
   */
  public static CubicBezierControl createEnd(Vector2D_R p2, Vector2D_R p3) {
    return new CubicBezierControl(p3.clone(), p3.clone()
                                                .multiply(2)
                                                .subtract(p2));
  }
}
