package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

import org.ejml.data.DMatrixRMaj;

public class CubicBezierSegment extends SplineSegment {
  private static final double[][] POSITION_COEFFICIENTS = {
      // @format:off
      /* t^0 */ {  1,  0,  0,  0 },
      /* t^1 */ { -3,  3,  0,  0 },
      /* t^2 */ {  3, -6,  3,  0 },
      /* t^3 */ { -1,  3, -3,  1 },
      // @format:on
  };

  private static final double[][] VELOCITY_COEFFICIENTS = {
      // @format:off
      /* t^0 */ { -3,  3,  0,  0 },
      /* t^1 */ {  6,-12,  6,  0 },
      /* t^2 */ { -3,  9, -9,  3 },
      /* t^3 */ {  0,  0,  0,  0 },
      // @format:on
  };

  private static final double[][] ACCELERATION_COEFFICIENTS = {
      // @format:off
      /* t^0 */ {  6,-12,  6,  0 },
      /* t^1 */ { -6, 18,-18,  6 },
      /* t^2 */ {  0,  0,  0,  0 },
      /* t^3 */ {  0,  0,  0,  0 },
      // @format:on
  };

  private static final double[][] JOLT_COEFFICIENTS = {
      // @format:off
      /* t^0 */ { -6, 18,-18,  6 },
      /* t^1 */ {  0,  0,  0,  0 },
      /* t^2 */ {  0,  0,  0,  0 },
      /* t^3 */ {  0,  0,  0,  0 },
      // @format:on
  };

  private static final DMatrixRMaj POSITION_MATRIX     = new DMatrixRMaj(POSITION_COEFFICIENTS);
  private static final DMatrixRMaj VELOCITY_MATRIX     = new DMatrixRMaj(VELOCITY_COEFFICIENTS);
  private static final DMatrixRMaj ACCELERATION_MATRIX = new DMatrixRMaj(ACCELERATION_COEFFICIENTS);
  private static final DMatrixRMaj JOLT_MATRIX         = new DMatrixRMaj(JOLT_COEFFICIENTS);

  private final CubicBezierControl startControl;
  private final CubicBezierControl endControl;
  private int                        startModCount;
  private int                        endModCount;

  protected CubicBezierSegment(Vector2D_R p0, Vector2D_R p1, Vector2D_R p2, Vector2D_R p3) {
    this(CubicBezierControl.createStart(p0, p1), CubicBezierControl.createEnd(p2, p3));
  }

  protected CubicBezierSegment(CubicBezierControl startControl, CubicBezierControl endControl) {
    super(3);
    this.startControl = startControl;
    this.endControl = endControl;
    startModCount = -1;
    endModCount = -1;
  }

  @Override
  public Vector2D_R getPosition(double t) {
    return compute(t, POSITION_MATRIX);
  }

  @Override
  public Vector2D_R getVelocity(double t) {
    return compute(t, VELOCITY_MATRIX);
  }

  @Override
  public Vector2D_R getAcceleration(double t) {
    return compute(t, ACCELERATION_MATRIX);
  }

  @Override
  public Vector2D_R getJolt(double t) {
    return compute(t, JOLT_MATRIX);
  }

  @Override
  public Vector2D_R getControlPoint(int index) {
    return switch (index) {
      case 0 -> startControl.getP0();
      case 1 -> startControl.getP1();
      case 2 -> endControl.getP2();
      case 3 -> endControl.getP3();
      default -> throw new IndexOutOfBoundsException(index);
    };
  }

  @Override
  public void setControlPoint(int index, Vector2D_R control) {
    switch (index) {
      case 0 -> startControl.setP0(control);
      case 1 -> startControl.setP1(control);
      case 2 -> endControl.setP2(control);
      case 3 -> endControl.setP3(control);
      default -> throw new IndexOutOfBoundsException(index);
    }
  }

  @Override
  protected void fillControlMatrix() {
    int startMod = startControl.getModCount();
    if (startMod != startModCount) {
      startModCount = startMod;
      Vector2D_R p0 = startControl.getP0Raw();
      Vector2D_R p1 = startControl.getP1Raw();
      controlMatrix.set(0, 0, p0.getX());
      controlMatrix.set(0, 1, p0.getY());
      controlMatrix.set(0, 2, p0.getR());
      controlMatrix.set(1, 0, p1.getX());
      controlMatrix.set(1, 1, p1.getY());
      controlMatrix.set(1, 2, p1.getR());
    }
    int endMod = startControl.getModCount();
    if (endMod != endModCount) {
      endModCount = endMod;
      Vector2D_R p2 = endControl.getP2();
      Vector2D_R p3 = endControl.getP3Raw();
      controlMatrix.set(2, 0, p2.getX());
      controlMatrix.set(2, 1, p2.getY());
      controlMatrix.set(2, 2, p2.getR());
      controlMatrix.set(3, 0, p3.getX());
      controlMatrix.set(3, 1, p3.getY());
      controlMatrix.set(3, 2, p3.getR());
    }
  }

  @Override
  public CubicBezierControl getStartControl() {
    return startControl;
  }

  @Override
  public CubicBezierControl getEndControl() {
    return endControl;
  }
}
