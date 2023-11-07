package org.victorrobotics.dtlib.math.kinematics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import java.util.Objects;

/** Represents the state of one swerve module. */
public class SwerveModulePosition
    implements Comparable<SwerveModulePosition>, Interpolatable<SwerveModulePosition>, Cloneable {
  /** Distance measured by the wheel of the module. */
  public double distance;

  /** Angle of the module. */
  public Rotation2d angle = new Rotation2d();

  /** Constructs a SwerveModulePosition with zeros for distance and angle. */
  public SwerveModulePosition() {}

  /**
   * Constructs a SwerveModulePosition.
   *
   * @param distance
   *        The distance measured by the wheel of the module.
   * @param angle
   *        The angle of the module.
   */
  public SwerveModulePosition(double distance, Rotation2d angle) {
    this.distance = distance;
    this.angle = angle;
  }

  @Override
  public boolean equals(Object obj) {
    return this == obj || (obj instanceof SwerveModulePosition other
        && Math.abs(other.distance - distance) < 1E-9 && angle.equals(other.angle));
  }

  @Override
  public int hashCode() {
    return Objects.hash(distance, angle);
  }

  /**
   * Compares two swerve module positions. One swerve module is "greater" than
   * the other if its distance is higher than the other.
   *
   * @param other
   *        The other swerve module.
   *
   * @return 1 if this is greater, 0 if both are equal, -1 if other is greater.
   */
  @Override
  public int compareTo(SwerveModulePosition other) {
    return Double.compare(distance, other.distance);
  }

  @Override
  public String toString() {
    return String.format("SwerveModulePosition(Distance: %.2f m, Angle: %s)", distance, angle);
  }

  @Override
  public SwerveModulePosition clone() {
    try {
      return (SwerveModulePosition) super.clone();
    } catch (CloneNotSupportedException e) {
      throw new IllegalStateException(e);
    }
  }

  @Override
  public SwerveModulePosition interpolate(SwerveModulePosition endValue, double t) {
    return new SwerveModulePosition(MathUtil.interpolate(distance, endValue.distance, t),
                                    angle.interpolate(endValue.angle, t));
  }
}
