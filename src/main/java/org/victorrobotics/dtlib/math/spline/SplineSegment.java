package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.DTVector2dR;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public abstract class DTCurve {
  protected final int         degree;
  protected final DMatrixRMaj controlMatrix;
  protected final DMatrixRMaj coefficientMatrix;
  protected final DMatrixRMaj resultMatrix;
  protected final DMatrixRMaj tMatrix;

  protected DTCurve(int degree) {
    int pointCount = degree + 1;
    this.degree = degree;
    controlMatrix = new DMatrixRMaj(pointCount, 3);
    resultMatrix = new DMatrixRMaj(1, 3);
    coefficientMatrix = new DMatrixRMaj(pointCount, pointCount);
    tMatrix = new DMatrixRMaj(1, pointCount);
  }

  public abstract DTVector2dR getControlPoint(int index);

  public abstract DTCurveControl getStartControl();

  public abstract DTCurveControl getEndControl();

  public abstract void setControlPoint(int index, DTVector2dR control);

  protected void fillControlMatrix() {
    for (int i = 0; i <= degree; i++) {
      DTVector2dR controlPoint = getControlPoint(i);
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

  protected DTVector2dR compute(double t, DMatrixRMaj coefficients) {
    fillControlMatrix();
    fillTMatrix(t);

    CommonOps_DDRM.mult(tMatrix, coefficients, coefficientMatrix);
    CommonOps_DDRM.mult(coefficientMatrix, controlMatrix, resultMatrix);
    return new DTVector2dR(resultMatrix.get(0), resultMatrix.get(1), resultMatrix.get(2));
  }

  public abstract DTVector2dR getPosition(double t);

  public abstract DTVector2dR getVelocity(double t);

  public abstract DTVector2dR getAcceleration(double t);

  public abstract DTVector2dR getJolt(double t);
}
