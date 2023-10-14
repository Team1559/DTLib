package org.victorrobotics.dtlib.math.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DTVector2d implements Cloneable {
  protected static final Rotation2d ZERO_ROTATION = new Rotation2d();

  protected double x;
  protected double y;

  public DTVector2d() {}

  public DTVector2d(double x, double y) {
    this.x = x;
    this.y = y;
  }

  public DTVector2d(Translation2d translation) {
    x = translation.getX();
    y = translation.getY();
  }

  public DTVector2d(Pose2d pose) {
    x = pose.getX();
    y = pose.getY();
  }

  public DTVector2d(ChassisSpeeds speeds) {
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

  public DTVector2d set(DTVector2d other) {
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

  public DTVector2d invert() {
    x = -x;
    y = -y;
    return this;
  }

  public DTVector2d add(DTVector2d other) {
    x += other.x;
    y += other.y;
    return this;
  }

  public DTVector2d subtract(DTVector2d other) {
    x -= other.x;
    y -= other.y;
    return this;
  }

  public DTVector2d multiply(double scalar) {
    x *= scalar;
    y *= scalar;
    return this;
  }

  public DTVector2d divide(double scalar) {
    return multiply(1 / scalar);
  }

  public DTVector2d normalize(double d) {
    return multiply(d / getNorm());
  }

  @Override
  public DTVector2d clone() {
    try {
      return (DTVector2d) super.clone();
    } catch (CloneNotSupportedException e) {
      throw new IllegalStateException(e);
    }
  }
}
