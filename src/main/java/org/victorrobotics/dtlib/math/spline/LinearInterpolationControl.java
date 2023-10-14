package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

public class LinearInterpolationControl extends SplineControl {
  private Vector2D_R position;

  public LinearInterpolationControl() {
    position = new Vector2D_R();
  }

  public LinearInterpolationControl(Vector2D_R position) {
    this.position = position.clone();
  }

  public Vector2D_R getPosition() {
    return position.clone();
  }

  protected Vector2D_R getPosRaw() {
    return position;
  }

  public void setPosition(Vector2D_R position) {
    this.position.set(position);
  }
}
