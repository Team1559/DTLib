package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public abstract class SplineSegment {
  protected final int         degree;
  protected final DMatrixRMaj controlMatrix;
  protected final DMatrixRMaj coefficientMatrix;
  protected final DMatrixRMaj resultMatrix;
  protected final DMatrixRMaj tMatrix;

  protected SplineSegment(int degree) {
    int pointCount = degree + 1;
    this.degree = degree;
    controlMatrix = new DMatrixRMaj(pointCount, 3);
    resultMatrix = new DMatrixRMaj(1, 3);
    coefficientMatrix = new DMatrixRMaj(pointCount, pointCount);
    tMatrix = new DMatrixRMaj(1, pointCount);
  }

  public abstract Vector2D_R getControlPoint(int index);

  public abstract SplineControl getStartControl();

  public abstract SplineControl getEndControl();

  public abstract void setControlPoint(int index, Vector2D_R control);

  protected void fillControlMatrix() {
    for (int i = 0; i <= degree; i++) {
      Vector2D_R controlPoint = getControlPoint(i);
      controlMatrix.set(i, 0, controlPoint.getX());
      controlMatrix.set(i, 1, controlPoint.getY());
      controlMatrix.set(i, 2, controlPoint.getR());
    }
  }

  protected void fillTMatrix(double t) {
    for (int i = 0; i <= degree; i++) {
      tMatrix.set(i, Math.pow(t, i));
    }
  }

  protected Vector2D_R compute(double t, DMatrixRMaj coefficients) {
    fillControlMatrix();
    fillTMatrix(t);

    CommonOps_DDRM.mult(tMatrix, coefficients, coefficientMatrix);
    CommonOps_DDRM.mult(coefficientMatrix, controlMatrix, resultMatrix);
    return new Vector2D_R(resultMatrix.get(0), resultMatrix.get(1), resultMatrix.get(2));
  }

  public abstract Vector2D_R getPosition(double t);

  public abstract Vector2D_R getVelocity(double t);

  public abstract Vector2D_R getAcceleration(double t);

  public abstract Vector2D_R getJolt(double t);
}
