package org.victorrobotics.dtlib.math.kinematics;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Objects;

/** Represents the state of one swerve module. */
public class SwerveModuleState {
  public double     speed;
  public Rotation2d angle;

  /**
   * Constructs a SwerveModuleState with zeros for speed and angle.
   */
  public SwerveModuleState() {
    speed = 0;
    angle = new Rotation2d();
  }

  /**
   * Constructs a SwerveModuleState.
   *
   * @param speed
   *        The speed of the wheel of the module.
   * @param angle
   *        The angle of the module.
   */
  public SwerveModuleState(double speed, Rotation2d angle) {
    this.speed = speed;
    this.angle = angle;
  }

  @Override
  @SuppressWarnings("java:S1244") // floating point equality test
  public boolean equals(Object obj) {
    return this == obj || (obj instanceof SwerveModuleState other && speed == other.speed
        && Objects.equals(angle, other.angle));
  }

  @Override
  public int hashCode() {
    return Objects.hash(speed, angle);
  }

  @Override
  public String toString() {
    return String.format("SwerveModuleState(Speed: %.2f m/s, Angle: %s)", speed, angle);
  }
}
