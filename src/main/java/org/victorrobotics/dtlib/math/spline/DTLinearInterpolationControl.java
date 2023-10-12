package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.DTVector2dR;

public class DTLinearInterpolationControl extends DTCurveControl {
  private DTVector2dR position;

  public DTLinearInterpolationControl() {
    position = new DTVector2dR();
  }

  public DTLinearInterpolationControl(DTVector2dR position) {
    this.position = position.clone();
  }

  public DTVector2dR getPosition() {
    return position.clone();
  }

  protected DTVector2dR getPosRaw() {
    return position;
  }

  public void setPosition(DTVector2dR position) {
    this.position.set(position);
  }
}
