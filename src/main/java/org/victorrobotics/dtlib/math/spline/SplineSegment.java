package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

/**
 * A parent class for curve segments that are calculated from control points.
 */
public abstract class SplineSegment {
  private final int         degree;
  private final DMatrixRMaj controlMatrix;
  private final DMatrixRMaj coefficientMatrix;
  private final DMatrixRMaj resultMatrix;
  private final DMatrixRMaj tMatrix;

  /**
   * Constructs a new SplineSegment.
   *
   * @param degree the degree of the curve (the order of its polynomial)
   */
  protected SplineSegment(int degree) {
    int pointCount = degree + 1;
    this.degree = degree;
    controlMatrix = new DMatrixRMaj(pointCount, 3);
    resultMatrix = new DMatrixRMaj(1, 3);
    coefficientMatrix = new DMatrixRMaj(pointCount, pointCount);
    tMatrix = new DMatrixRMaj(1, pointCount);
  }

  /**
   * Get the control point at the specified index.
   *
   * @param index the control point's index
   * @return a copy of the control point's vector
   * @throws IndexOutOfBoundsException if index is negative or greater than the
   *         degree of this curve segment
   */
  public abstract Vector2D_R getControlPoint(int index);

  /**
   * Sets the control point at the specified index to a new value.
   *
   * @param index the control point's index
   * @param control the new vector for that point
   * @throws IndexOutOfBoundsException if index is negative or greater than the
   *         degree of this curve segment
   */
  public abstract void setControlPoint(int index, Vector2D_R control);

  /**
   * @return the control object containing the first half of the control points
   */
  public abstract SplineControl getStartControl();

  /**
   * @return the control object containing the last half of the control points
   */
  public abstract SplineControl getEndControl();

  /**
   * Fills the control points into the given matrix to prepare for computation.
   *
   * @param controlMatrix the matrix to fill
   */
  protected void fillControlMatrix(DMatrixRMaj controlMatrix) {
    for (int i = 0; i <= degree; i++) {
      Vector2D_R controlPoint = getControlPoint(i);
      controlMatrix.set(i, 0, controlPoint.getX());
      controlMatrix.set(i, 1, controlPoint.getY());
      controlMatrix.set(i, 2, controlPoint.getR());
    }
  }

  /**
   * Computes a point along this curve segment, using the given coefficients in
   * a matrix multiplication.
   *
   * @param t the "time" to get the point (usually between 0 and 1)
   * @param coefficients the coefficients representing the desired data
   * @return the vector of the desired type at the specified "time"
   */
  protected Vector2D_R compute(double t, DMatrixRMaj coefficients) {
    fillControlMatrix(controlMatrix);
    for (int i = 0; i <= degree; i++) {
      tMatrix.set(i, Math.pow(t, i));
    }

    CommonOps_DDRM.mult(tMatrix, coefficients, coefficientMatrix);
    CommonOps_DDRM.mult(coefficientMatrix, controlMatrix, resultMatrix);
    return new Vector2D_R(resultMatrix.get(0), resultMatrix.get(1), resultMatrix.get(2));
  }

  /**
   * Get the position of a point on this curve segment.
   *
   * @param t the "time" to get the point (usually between 0 and 1)
   * @return the position at the specified time
   * @see #compute(double, DMatrixRMaj)
   */
  public abstract Vector2D_R getPosition(double t);

  /**
   * Get the velocity of a point on this curve segment.
   *
   * @param t the "time" to get the point (usually between 0 and 1)
   * @return the velocity at the specified time
   * @see #compute(double, DMatrixRMaj)
   */
  public abstract Vector2D_R getVelocity(double t);

  /**
   * Get the acceleration of a point on this curve segment.
   *
   * @param t the "time" to get the point (usually between 0 and 1)
   * @return the acceleration at the specified time
   * @see #compute(double, DMatrixRMaj)
   */
  public abstract Vector2D_R getAcceleration(double t);

  /**
   * Get the jolt of a point on this curve segment.
   *
   * @param t the "time" to get the point (usually between 0 and 1)
   * @return the jolt at the specified time
   * @see #compute(double, DMatrixRMaj)
   */
  public abstract Vector2D_R getJolt(double t);
}
