package org.victorrobotics.dtlib.drivetrain;

import org.victorrobotics.dtlib.math.geometry.DTVector2dR;

public final class DTVelocityLimit {
  public final double maxVelocityTranslation;
  public final double maximumAngularVelocity;
  public final double minimumLinearVelocity;
  public final double minimumAngularVelocity;

  public DTVelocityLimit() {
    this(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
  }

  public DTVelocityLimit(double translation, double rotation) {
    this(Double.NaN, translation, Double.NaN, rotation);
  }

  public DTVelocityLimit(double minTranslation, double maxTranslation, double minRotation,
      double maxRotation) {
    maxVelocityTranslation = Double.isFinite(maxTranslation) ? Math.abs(maxTranslation)
      : Double.NaN;
    maximumAngularVelocity = Double.isFinite(maxRotation) ? Math.abs(maxRotation) : Double.NaN;
    minimumLinearVelocity = Double.isFinite(minTranslation) ? Math.abs(minTranslation) : Double.NaN;
    minimumAngularVelocity = Double.isFinite(minRotation) ? Math.abs(minRotation) : Double.NaN;

    if ((!Double.isNaN(maxVelocityTranslation) && !Double.isNaN(minimumLinearVelocity)
        && minimumLinearVelocity > maxVelocityTranslation)
        || (!Double.isNaN(maximumAngularVelocity) && !Double.isNaN(minimumAngularVelocity)
            && minimumAngularVelocity > maximumAngularVelocity)) {
      throw new IllegalArgumentException("Min velocity must be less than max velocity");
    }
  }

  public boolean apply(DTVector2dR speeds) {
    boolean change = false;

    double translationVelocity = speeds.getNorm();
    if (!Double.isNaN(maxVelocityTranslation) && translationVelocity > maxVelocityTranslation) {
      speeds.multiply(maxVelocityTranslation / translationVelocity);
      change = true;
    }

    double rotationVelocity = Math.abs(speeds.getR());
    if (!Double.isNaN(maximumAngularVelocity) && rotationVelocity > maximumAngularVelocity) {
      speeds.multiply(maximumAngularVelocity / rotationVelocity);
      change = true;
    }

    return change;
  }
}
