package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.DTVector2dR;

public class DTLinearInterpolationCurve extends DTCurve {
  private final DTLinearInterpolationControl startControl;
  private final DTLinearInterpolationControl endControl;

  public DTLinearInterpolationCurve(DTVector2dR p0, DTVector2dR p1) {
    this(new DTLinearInterpolationControl(p0), new DTLinearInterpolationControl(p1));
  }

  public DTLinearInterpolationCurve(DTLinearInterpolationControl startControl,
                                    DTLinearInterpolationControl endControl) {
    super(1);
    this.startControl = startControl;
    this.endControl = endControl;
  }

  @Override
  public DTVector2dR getPosition(double t) {
    DTVector2dR start = startControl.getPosition();
    return start.add(endControl.getPosition()
                               .subtract(start)
                               .multiply(t));
  }

  @Override
  public DTVector2dR getVelocity(double t) {
    return endControl.getPosition()
                     .subtract(startControl.getPosRaw());
  }

  @Override
  public DTVector2dR getAcceleration(double t) {
    return new DTVector2dR();
  }

  @Override
  public DTVector2dR getJolt(double t) {
    return new DTVector2dR();
  }

  @Override
  public DTVector2dR getControlPoint(int index) {
    return switch (index) {
      case 0 -> startControl.getPosition();
      case 1 -> endControl.getPosition();
      default -> throw new IndexOutOfBoundsException(index);
    };
  }

  @Override
  public void setControlPoint(int index, DTVector2dR control) {
    switch (index) {
      case 0 -> startControl.setPosition(control);
      case 1 -> endControl.setPosition(control);
      default -> throw new IndexOutOfBoundsException(index);
    }
  }

  @Override
  public DTLinearInterpolationControl getStartControl() {
    return startControl;
  }

  @Override
  public DTLinearInterpolationControl getEndControl() {
    return endControl;
  }
}
