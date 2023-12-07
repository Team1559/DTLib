package org.victorrobotics.dtlib.math.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Vector2D_R extends Vector2D {
  protected double r;

  public Vector2D_R() {}

  public Vector2D_R(double x, double y, double r) {
    super(x, y);
    this.r = r;
  }

  public Vector2D_R(Translation2d translation, Rotation2d rotation) {
    super(translation);
    r = rotation.getRadians();
  }

  public Vector2D_R(Pose2d pose) {
    super(pose);
    r = pose.getRotation()
            .getRadians();
  }

  public Vector2D_R(ChassisSpeeds speeds) {
    super(speeds);
    r = speeds.omegaRadiansPerSecond;
  }

  public final double getR() {
    return r;
  }

  public final void setR(double r) {
    this.r = r;
  }

  public final Rotation2d toRotation2d() {
    return Rotation2d.fromRadians(r);
  }

  @Override
  public final Pose2d toPose2d() {
    return new Pose2d(x, y, toRotation2d());
  }

  @Override
  public final ChassisSpeeds toChassisSpeeds() {
    return new ChassisSpeeds(x, y, r);
  }

  @Override
  public Vector2D_R set(Vector2D other) {
    super.set(other);
    if (other instanceof Vector2D_R) {
      r = ((Vector2D_R) other).r;
    }
    return this;
  }

  @Override
  public Vector2D_R invert() {
    super.invert();
    r = -r;
    return this;
  }

  @Override
  public Vector2D_R add(Vector2D other) {
    super.add(other);
    if (other instanceof Vector2D_R) {
      r += ((Vector2D_R) other).r;
    }
    return this;
  }

  @Override
  public Vector2D_R subtract(Vector2D other) {
    super.subtract(other);
    if (other instanceof Vector2D_R) {
      r -= ((Vector2D_R) other).r;
    }
    return this;
  }

  @Override
  public Vector2D_R multiply(double scalar) {
    super.multiply(scalar);
    r *= scalar;
    return this;
  }

  @Override
  public Vector2D_R divide(double scalar) {
    super.divide(scalar);
    return this;
  }

  @Override
  public Vector2D_R normalize(double d) {
    super.normalize(d);
    return this;
  }

  @Override
  public Vector2D_R clone() {
    return (Vector2D_R) super.clone();
  }
}
