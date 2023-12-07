package org.victorrobotics.dtlib.math.kinematics;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Objects;

/** Represents the state of one swerve module. */
public class SwerveModuleState {
  private static final double HALF_PI = Math.PI / 2;

  public double     speed;
  public Rotation2d heading;

  /**
   * Constructs a SwerveModuleState with zeros for speed and angle.
   */
  public SwerveModuleState() {
    speed = 0;
    heading = new Rotation2d();
  }

  /**
   * Constructs a SwerveModuleState.
   *
   * @param speed The speed of the wheel of the module.
   * @param angle The angle of the module.
   */
  public SwerveModuleState(double speed, Rotation2d angle) {
    this.speed = speed;
    this.heading = angle;
  }

  public void optimize(Rotation2d currentHeading) {
    double target = heading.getRadians();

    double currentAngle = currentHeading.getRadians();
    while (target - currentAngle > HALF_PI) {
      target -= Math.PI;
      speed = -speed;
    }
    while (currentAngle - target > HALF_PI) {
      target += Math.PI;
      speed = -speed;
    }

    heading = Rotation2d.fromRadians(target);
  }

  @Override
  @SuppressWarnings("java:S1244") // floating point equality test
  public boolean equals(Object obj) {
    return this == obj || (obj instanceof SwerveModuleState other && speed == other.speed
        && Objects.equals(heading, other.heading));
  }

  @Override
  public int hashCode() {
    return Objects.hash(speed, heading);
  }

  @Override
  public String toString() {
    return String.format("SwerveModuleState(Speed: %.2f m/s, Angle: %s)", speed, heading);
  }
}
