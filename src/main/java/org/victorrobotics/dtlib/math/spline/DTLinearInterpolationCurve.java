package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.DTVector2DR;

public class DTLinearInterpolationCurve extends DTCurve {
  private final DTLinearInterpolationControl startControl;
  private final DTLinearInterpolationControl endControl;

  public DTLinearInterpolationCurve(DTVector2DR p0, DTVector2DR p1) {
    this(new DTLinearInterpolationControl(p0), new DTLinearInterpolationControl(p1));
  }

  public DTLinearInterpolationCurve(DTLinearInterpolationControl startControl,
                                    DTLinearInterpolationControl endControl) {
    super(1);
    this.startControl = startControl;
    this.endControl = endControl;
  }

  @Override
  public DTVector2DR getPosition(double t) {
    return startControl.getPosRaw()
                       .interpolate(endControl.getPosRaw(), t);
  }

  @Override
  public DTVector2DR getVelocity(double t) {
    return endControl.getPosition()
                     .subtract(startControl.getPosRaw());
  }

  @Override
  public DTVector2DR getAcceleration(double t) {
    return new DTVector2DR();
  }

  @Override
  public DTVector2DR getJolt(double t) {
    return new DTVector2DR();
  }

  @Override
  public DTVector2DR getControlPoint(int index) {
    return switch (index) {
      case 0 -> startControl.getPosition();
      case 1 -> endControl.getPosition();
      default -> throw new IndexOutOfBoundsException(index);
    };
  }

  @Override
  public void setControlPoint(int index, DTVector2DR control) {
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
