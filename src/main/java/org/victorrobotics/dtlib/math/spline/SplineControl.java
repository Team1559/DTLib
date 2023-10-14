package org.victorrobotics.dtlib.math.spline;

/**
 * A generic superclass for spline segment control objects.
 */
public abstract class SplineControl {
  /**
   * The number of times these control vectors have been modified. Used to
   * determine when to refill spline matrices.
   */
  protected int modCount;

  /**
   * Get the number of times these control vectors have been modified.
   *
   * @return the modification count
   */
  public int getModCount() {
    return modCount;
  }
}
