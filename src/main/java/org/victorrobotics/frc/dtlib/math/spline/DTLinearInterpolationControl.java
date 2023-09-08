package org.victorrobotics.frc.dtlib.math.spline;

import org.victorrobotics.frc.dtlib.math.geometry.DTVector2DR;

public class DTLinearInterpolationControl extends DTCurveControl {
  private DTVector2DR position;

  public DTLinearInterpolationControl() {
    position = new DTVector2DR();
  }

  public DTLinearInterpolationControl(DTVector2DR position) {
    this.position = position.clone();
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
