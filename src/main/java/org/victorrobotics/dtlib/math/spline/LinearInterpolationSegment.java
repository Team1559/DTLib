package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

public class LinearInterpolationSegment extends SplineSegment {
  private final LinearInterpolationControl startControl;
  private final LinearInterpolationControl endControl;

  public LinearInterpolationSegment(Vector2D_R p0, Vector2D_R p1) {
    this(new LinearInterpolationControl(p0), new LinearInterpolationControl(p1));
  }

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
