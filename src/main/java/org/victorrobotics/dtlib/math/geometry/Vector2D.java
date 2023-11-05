package org.victorrobotics.dtlib.math.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Vector2D implements Cloneable {
  protected static final Rotation2d ZERO_ROTATION = new Rotation2d();

  protected double x;
  protected double y;

  public Vector2D() {}

  public Vector2D(double x, double y) {
    this.x = x;
    this.y = y;
  }

  public Vector2D(Translation2d translation) {
    x = translation.getX();
    y = translation.getY();
  }

  public Vector2D(Pose2d pose) {
    x = pose.getX();
    y = pose.getY();
  }

  public Vector2D(ChassisSpeeds speeds) {
    x = speeds.vxMetersPerSecond;
    y = speeds.vyMetersPerSecond;
  }

  public final double getX() {
    return x;
  }

  public final double getY() {
    return y;
  }

  public final void setX(double x) {
    this.x = x;
  }

  public final void setY(double y) {
    this.y = y;
  }

  public final double getNorm() {
    return Math.hypot(x, y);
  }

  public final double theta() {
    return Math.atan2(y, x);
  }

  public final Translation2d toTranslation2d() {
    return new Translation2d(x, y);
  }

  public Vector2D set(Vector2D other) {
    x = other.x;
    y = other.y;
    return this;
  }

  public Pose2d toPose2d() {
    return new Pose2d(x, y, ZERO_ROTATION);
  }

  public ChassisSpeeds toChassisSpeeds() {
    return new ChassisSpeeds(x, y, 0);
  }

  public Vector2D invert() {
    x = -x;
    y = -y;
    return this;
  }

  public Vector2D add(Vector2D other) {
    x += other.x;
    y += other.y;
    return this;
  }

  public Vector2D subtract(Vector2D other) {
    x -= other.x;
    y -= other.y;
    return this;
  }

  public Vector2D multiply(double scalar) {
    x *= scalar;
    y *= scalar;
    return this;
  }

  public Vector2D divide(double scalar) {
    return multiply(1 / scalar);
  }

  public Vector2D normalize(double d) {
    return multiply(d / getNorm());
  }

  @Override
  public Vector2D clone() {
    try {
      return (Vector2D) super.clone();
    } catch (CloneNotSupportedException e) {
      throw new IllegalStateException(e);
    }
  }
}
