package org.victorrobotics.frc.dtlib.math.geometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DTVector2DR implements Cloneable, Interpolatable<DTVector2DR> {
  protected double x;
  protected double y;
  protected double r;

  public DTVector2DR() {}

  public DTVector2DR(double x, double y, double r) {
    this.x = x;
    this.y = y;
    this.r = r;
  }

  public DTVector2DR(Translation2d translation) {
    x = translation.getX();
    y = translation.getY();
  }

  public DTVector2DR(Pose2d pose) {
    x = pose.getX();
    y = pose.getY();
    r = pose.getRotation()
            .getRadians();
  }

  public DTVector2DR(ChassisSpeeds speeds) {
    x = speeds.vxMetersPerSecond;
    y = speeds.vyMetersPerSecond;
    r = speeds.omegaRadiansPerSecond;
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getR() {
    return r;
  }

  public void setX(double x) {
    this.x = x;
  }

  public void setY(double y) {
    this.y = y;
  }

  public void setR(double r) {
    this.r = r;
  }

  public void set(DTVector2DR other) {
    x = other.x;
    y = other.y;
    r = other.r;
  }

  public Translation2d toTranslation2d() {
    return new Translation2d(x, y);
  }

  public Pose2d toPose2d() {
    return new Pose2d(x, y, Rotation2d.fromRadians(r));
  }

  public ChassisSpeeds toChassisSpeeds() {
    return new ChassisSpeeds(x, y, r);
  }

  public DTVector2DR invert() {
    x = -x;
    y = -y;
    r = -r;
    return this;
  }

  public DTVector2DR add(DTVector2DR other) {
    x += other.x;
    y += other.y;
    r += other.r;
    return this;
  }

  public DTVector2DR subtract(DTVector2DR other) {
    x -= other.x;
    y -= other.y;
    r -= other.r;
    return this;
  }

  public DTVector2DR multiply(double scalar) {
    x *= scalar;
    y *= scalar;
    r *= scalar;
    return this;
  }

  public DTVector2DR divide(double scalar) {
    return multiply(1 / scalar);
  }

  public double hypotenuse() {
    return Math.hypot(x, y);
  }

  public double scaleX(double newX) {
    double scalar = newX / x;
    if (Double.isFinite(scalar)) {
      multiply(scalar);
      return scalar;
    }
    return 1;
  }

  public double scaleY(double newY) {
    double scalar = newY / y;
    if (Double.isFinite(scalar)) {
      multiply(scalar);
      return scalar;
    }
    return 1;
  }

  public double scaleR(double newR) {
    double scalar = newR / r;
    if (Double.isFinite(scalar)) {
      multiply(scalar);
      return scalar;
    }
    return 1;
  }

  public double scaleHypotenuse(double newHypot) {
    double scalar = newHypot / hypotenuse();
    if (Double.isFinite(scalar)) {
      multiply(scalar);
      return scalar;
    }
    return 1;
  }

  public double scaleDownHypotenuse(double newHypot) {
    double scalar = newHypot / hypotenuse();
    if (Double.isFinite(scalar) && Math.abs(scalar) < 1) {
      multiply(scalar);
      return scalar;
    }
    return 1;
  }

  public double theta() {
    return Math.atan2(y, x);
  }

  @Override
  public DTVector2DR clone() {
    try {
      return (DTVector2DR) super.clone();
    } catch (CloneNotSupportedException e) {
      // Should never happen, we are Cloneable
      throw new IllegalStateException(e);
    }
  }

  @Override
  public String toString() {
    return String.format("[x=%.2f y=%.2f r=%.2f h=%.2f]", x, y, r, hypotenuse());
  }

  @Override
  public DTVector2DR interpolate(DTVector2DR endValue, double t) {
    t = MathUtil.clamp(t, 0, 1);
    double newX = x + (endValue.x - x) * t;
    double newY = y + (endValue.y - y) * t;
    double newR = r + (endValue.r - r) * t;
    return new DTVector2DR(newX, newY, newR);
  }
}
