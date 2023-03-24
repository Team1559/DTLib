package org.victorrobotics.frc.dtlib.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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
        minimumLinearVelocity = Double.isFinite(minTranslation) ? Math.abs(minTranslation)
                : Double.NaN;
        minimumAngularVelocity = Double.isFinite(minRotation) ? Math.abs(minRotation) : Double.NaN;

        if ((!Double.isNaN(maxVelocityTranslation) && !Double.isNaN(minimumLinearVelocity)
                && minimumLinearVelocity > maxVelocityTranslation)
                || (!Double.isNaN(maximumAngularVelocity) && !Double.isNaN(minimumAngularVelocity)
                        && minimumAngularVelocity > maximumAngularVelocity)) {
            throw new IllegalArgumentException("Min velocity must be less than max velocity");
        }
    }

    public boolean apply(ChassisSpeeds speeds) {
        boolean changed = false;

        if (!Double.isNaN(maxVelocityTranslation)) {
            Translation2d translationVelocity = new Translation2d(speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond);
            double translationVelocityMagnitude = translationVelocity.getNorm();
            if (translationVelocityMagnitude > maxVelocityTranslation) {
                translationVelocity = translationVelocity.times(
                        maxVelocityTranslation / translationVelocityMagnitude);
                speeds.vxMetersPerSecond = translationVelocity.getX();
                speeds.vyMetersPerSecond = translationVelocity.getY();
                changed = true;
            }
        }

        if (!Double.isNaN(maximumAngularVelocity)) {
            if (speeds.omegaRadiansPerSecond > maximumAngularVelocity) {
                speeds.omegaRadiansPerSecond = maximumAngularVelocity;
                changed = true;
            } else if (speeds.omegaRadiansPerSecond < -maximumAngularVelocity) {
                speeds.omegaRadiansPerSecond = -maximumAngularVelocity;
                changed = true;
            }
        }

        return changed;
    }
}
