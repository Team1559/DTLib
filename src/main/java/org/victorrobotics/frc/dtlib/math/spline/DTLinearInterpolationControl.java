package org.victorrobotics.frc.dtlib.math.spline;

import org.victorrobotics.frc.dtlib.math.geometry.DTVector2DR;

public class DTLinearInterpolationControl implements DTCurveControl {
  private DTVector2DR position;
  private int         modCount;

  public DTLinearInterpolationControl() {
    position = new DTVector2DR();
  }

  public DTLinearInterpolationControl(DTVector2DR position) {
    this.position = position.clone();
  }

  @Override
  public int getModCount() {
    return modCount;
  }

  public DTVector2DR getPosition() {
    return position.clone();
  }

  protected DTVector2DR getPosRaw() {
    return position;
  }

  public void setPosition(DTVector2DR position) {
    this.position.set(position);
  }
}
