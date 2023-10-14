package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

import org.ejml.data.DMatrixRMaj;

public class QuinticBezierSegment extends SplineSegment {
  private static final double[][] POSITION_COEFFICIENTS = {
      // @format:off
      /* t^0 */ {   1,   0,   0,   0,   0,   0 },
      /* t^1 */ {  -5,   5,   0,   0,   0,   0 },
      /* t^2 */ {  10, -20,  10,   0,   0,   0 },
      /* t^3 */ { -10,  30, -30,  10,   0,   0 },
      /* t^4 */ {   5, -20,  30, -20,   5,   0 },
      /* t^5 */ {  -1,   5, -10,  10,  -5,   1 },
      // @format:on
  };

  private static final double[][] VELOCITY_COEFFICIENTS = {
      // @format:off
      /* t^0 */ {  -5,   5,   0,   0,   0,   0 },
      /* t^1 */ {  20, -40,  20,   0,   0,   0 },
      /* t^2 */ { -30,  90, -90,  30,   0,   0 },
      /* t^3 */ {  20, -80, 120, -80,  20,   0 },
      /* t^4 */ {  -5,  25, -50,  50, -25,   5 },
      /* t^5 */ {   0,   0,   0,   0,   0,   0 },
      // @format:on
  };

  private static final double[][] ACCELERATION_COEFFICIENTS = {
      // @format:off
      /* t^0 */ {  20, -40,  20,   0,   0,   0 },
      /* t^1 */ { -60, 180,-180,  60,   0,   0 },
      /* t^2 */ {  60,-240, 360,-240,  60,   0 },
      /* t^3 */ { -20, 100,-200, 200,-100,  20 },
      /* t^4 */ {   0,   0,   0,   0,   0,   0 },
      /* t^5 */ {   0,   0,   0,   0,   0,   0 },
      // @format:on
  };

  private static final double[][] JOLT_COEFFICIENTS = {
    // @format:off
    /* t^0 */ { -60, 180,-180,  60,   0,   0 },
    /* t^1 */ { 120,-480, 720,-480, 120,   0 },
    /* t^2 */ { -60, 300,-600, 600,-300,  60 },
    /* t^3 */ {   0,   0,   0,   0,   0,   0 },
    /* t^4 */ {   0,   0,   0,   0,   0,   0 },
    /* t^5 */ {   0,   0,   0,   0,   0,   0 },
    // @format:on
  };

  private static final DMatrixRMaj POSITION_MATRIX     = new DMatrixRMaj(POSITION_COEFFICIENTS);
  private static final DMatrixRMaj VELOCITY_MATRIX     = new DMatrixRMaj(VELOCITY_COEFFICIENTS);
  private static final DMatrixRMaj ACCELERATION_MATRIX = new DMatrixRMaj(ACCELERATION_COEFFICIENTS);
  private static final DMatrixRMaj JOLT_MATRIX         = new DMatrixRMaj(JOLT_COEFFICIENTS);

  private final QuinticBezierControl startControl;
  private final QuinticBezierControl endControl;
  private int                          startModCount;
  private int                          endModCount;

  protected QuinticBezierSegment(Vector2D_R p0, Vector2D_R p1, Vector2D_R p2, Vector2D_R p3,
                                 Vector2D_R p4, Vector2D_R p5) {
    this(QuinticBezierControl.createStart(p0, p1, p2),
         QuinticBezierControl.createEnd(p3, p4, p5));
  }

  protected QuinticBezierSegment(QuinticBezierControl startControl,
                                 QuinticBezierControl endControl) {
    super(5);
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
      case 2 -> startControl.getP2();
      case 3 -> endControl.getP3();
      case 4 -> endControl.getP4();
      case 5 -> endControl.getP5();
      default -> throw new IndexOutOfBoundsException(index);
    };
  }

  @Override
  public void setControlPoint(int index, Vector2D_R control) {
    switch (index) {
      case 0 -> startControl.setP0(control);
      case 1 -> startControl.setP1(control);
      case 2 -> startControl.setP2(control);
      case 3 -> endControl.setP3(control);
      case 4 -> endControl.setP4(control);
      case 5 -> endControl.setP5(control);
      default -> throw new IndexOutOfBoundsException(index);
    }
  }

  @Override
  protected void fillControlMatrix(DMatrixRMaj controlMatrix) {
    int startMod = startControl.getModCount();
    if (startMod != startModCount) {
      startModCount = startMod;
      Vector2D_R p0 = startControl.getP0Raw();
      Vector2D_R p1 = startControl.getP1Raw();
      Vector2D_R p2 = startControl.getP2Raw();
      controlMatrix.set(0, 0, p0.getX());
      controlMatrix.set(0, 1, p0.getY());
      controlMatrix.set(0, 2, p0.getR());
      controlMatrix.set(1, 0, p1.getX());
      controlMatrix.set(1, 1, p1.getY());
      controlMatrix.set(1, 2, p1.getR());
      controlMatrix.set(2, 0, p2.getX());
      controlMatrix.set(2, 1, p2.getY());
      controlMatrix.set(2, 2, p2.getR());
    }
    int endMod = startControl.getModCount();
    if (endMod != endModCount) {
      endModCount = endMod;
      Vector2D_R p3 = endControl.getP3();
      Vector2D_R p4 = endControl.getP4();
      Vector2D_R p5 = endControl.getP5Raw();
      controlMatrix.set(3, 0, p3.getX());
      controlMatrix.set(3, 1, p3.getY());
      controlMatrix.set(3, 2, p3.getR());
      controlMatrix.set(4, 0, p4.getX());
      controlMatrix.set(4, 1, p4.getY());
      controlMatrix.set(4, 2, p4.getR());
      controlMatrix.set(5, 0, p5.getX());
      controlMatrix.set(5, 1, p5.getY());
      controlMatrix.set(5, 2, p5.getR());
    }
  }

  @Override
  public QuinticBezierControl getStartControl() {
    return startControl;
  }

  @Override
  public QuinticBezierControl getEndControl() {
    return endControl;
  }
}
