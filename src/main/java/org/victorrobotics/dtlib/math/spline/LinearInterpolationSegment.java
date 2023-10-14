package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

/**
 * A single linearly-interpolated line segment which shares its control points
 * with segments immediately before and after it.
 */
public class LinearInterpolationSegment extends SplineSegment {
  private final LinearInterpolationControl startControl;
  private final LinearInterpolationControl endControl;

  /**
   * Constructs a DTLinearInterpolationCurve with the specified start and end
   * points.
   *
   * @param start the start point
   * @param end the end point
   */
  public LinearInterpolationSegment(Vector2D_R start, Vector2D_R end) {
    this(new LinearInterpolationControl(start), new LinearInterpolationControl(end));
  }

  /**
   * Constructs a DTlinearInterpolationCurve using the specified coontrol points
   * at the start and end.
   *
   * @param startControl the start point
   * @param endControl the end point
   */
  public LinearInterpolationSegment(LinearInterpolationControl startControl,
                                    LinearInterpolationControl endControl) {
    super(1);
    this.startControl = startControl;
    this.endControl = endControl;
  }

  @Override
  public Vector2D_R getPosition(double t) {
    Vector2D_R start = startControl.getPosition();
    return start.add(endControl.getPosition()
                               .subtract(start)
                               .multiply(t));
  }

  @Override
  public Vector2D_R getVelocity(double t) {
    return endControl.getPosition()
                     .subtract(startControl.getPosRaw());
  }

  @Override
  public Vector2D_R getAcceleration(double t) {
    return new Vector2D_R();
  }

  @Override
  public Vector2D_R getJolt(double t) {
    return new Vector2D_R();
  }

  @Override
  public Vector2D_R getControlPoint(int index) {
    return switch (index) {
      case 0 -> startControl.getPosition();
      case 1 -> endControl.getPosition();
      default -> throw new IndexOutOfBoundsException(index);
    };
  }

  @Override
  public void setControlPoint(int index, Vector2D_R control) {
    switch (index) {
      case 0 -> startControl.setPosition(control);
      case 1 -> endControl.setPosition(control);
      default -> throw new IndexOutOfBoundsException(index);
    }
  }

  @Override
  public LinearInterpolationControl getStartControl() {
    return startControl;
  }

  @Override
  public LinearInterpolationControl getEndControl() {
    return endControl;
  }
}
