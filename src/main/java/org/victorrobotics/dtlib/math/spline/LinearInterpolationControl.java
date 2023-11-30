package org.victorrobotics.dtlib.math.spline;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

/**
 * An object denoting a control point for two consequtive linear interpolations.
 */
public class LinearInterpolationControl extends SplineControl {
  private Vector2D_R position;

  /**
   * Constructs a LinearInterpolationControl at a default point.
   */
  public LinearInterpolationControl() {
    position = new Vector2D_R();
  }

  /**
   * Constructs a LinearInterpolationControl at the specified point.
   *
   * @param position the point at which to create the control
   */
  public LinearInterpolationControl(Vector2D_R position) {
    this.position = position.clone();
  }

  /**
   * @return a copy of this control point's position vector
   */
  public Vector2D_R getPosition() {
    return position.clone();
  }

  /**
   * @return the backing position vector for this control point
   */
  protected Vector2D_R getPosRaw() {
    return position;
  }

  /**
   * Sets this control point to a new position.
   *
   * @param position the new position for the point
   */
  public void setPosition(Vector2D_R position) {
    this.position.set(position);
  }
}
