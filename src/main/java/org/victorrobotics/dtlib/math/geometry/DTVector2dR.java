package org.victorrobotics.dtlib.math.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DTVector2dR extends DTVector2d {
  protected double r;

  public DTVector2dR() {}

  public DTVector2dR(double x, double y, double r) {
    super(x, y);
    this.r = r;
  }

  public DTVector2dR(Translation2d translation) {
    super(translation);
  }

  public DTVector2dR(Pose2d pose) {
    super(pose);
    r = pose.getRotation()
            .getRadians();
  }

  public DTVector2dR(ChassisSpeeds speeds) {
    super(speeds);
    r = speeds.omegaRadiansPerSecond;
  }

  public final double getR() {
    return r;
  }

  public final void setR(double r) {
    this.r = r;
  }

  @Override
  public final Pose2d toPose2d() {
    return new Pose2d(x, y, Rotation2d.fromRadians(r));
  }

  @Override
  public final ChassisSpeeds toChassisSpeeds() {
    return new ChassisSpeeds(x, y, r);
  }

  @Override
  public DTVector2dR set(DTVector2d other) {
    super.set(other);
    if (other instanceof DTVector2dR) {
      r = ((DTVector2dR) other).r;
    }
    return this;
  }

  @Override
  public DTVector2dR invert() {
    super.invert();
    r = -r;
    return this;
  }

  @Override
  public DTVector2dR add(DTVector2d other) {
    super.add(other);
    if (other instanceof DTVector2dR) {
      r += ((DTVector2dR) other).r;
    }
    return this;
  }

  @Override
  public DTVector2dR subtract(DTVector2d other) {
    super.subtract(other);
    if (other instanceof DTVector2dR) {
      r -= ((DTVector2dR) other).r;
    }
    return this;
  }

  @Override
  public DTVector2dR multiply(double scalar) {
    super.multiply(scalar);
    r *= scalar;
    return this;
  }

  @Override
  public DTVector2dR divide(double scalar) {
    super.divide(scalar);
    return this;
  }

  @Override
  public DTVector2dR normalize(double d) {
    super.normalize(d);
    return this;
  }

  @Override
  public DTVector2dR clone() {
    return (DTVector2dR) super.clone();
  }
}
