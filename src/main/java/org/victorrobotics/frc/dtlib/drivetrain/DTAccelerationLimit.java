package org.victorrobotics.frc.dtlib.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class DTAccelerationLimit {
    public final double maxAccelTranslation;
    public final double maxAccelRotation;

    public DTAccelerationLimit() {
        maxAccelTranslation = Double.NaN;
        maxAccelRotation = Double.NaN;
    }

    public DTAccelerationLimit(double translation, double rotation, double cycleLength) {
        maxAccelTranslation = Double.isFinite(translation) ? Math.abs(translation * cycleLength)
                : Double.NaN;
        maxAccelRotation = Double.isFinite(rotation) ? Math.abs(rotation * cycleLength)
                : Double.NaN;
    }

    public boolean apply(ChassisSpeeds newSpeeds, ChassisSpeeds previousSpeeds) {
        boolean changed = false;

        if (!Double.isNaN(maxAccelTranslation)) {
            Translation2d oldTranslation = new Translation2d(previousSpeeds.vxMetersPerSecond,
                    previousSpeeds.vyMetersPerSecond);
            Translation2d newTranslation = new Translation2d(newSpeeds.vxMetersPerSecond,
                    newSpeeds.vyMetersPerSecond);
            Translation2d translationAccel = newTranslation.minus(oldTranslation);
            double translationAccelMagnitude = translationAccel.getNorm();
            if (translationAccelMagnitude > maxAccelTranslation) {
                translationAccel = translationAccel.times(
                        maxAccelTranslation / translationAccelMagnitude);
                newSpeeds.vxMetersPerSecond = previousSpeeds.vxMetersPerSecond
                        + translationAccel.getX();
                newSpeeds.vyMetersPerSecond = previousSpeeds.vyMetersPerSecond
                        + translationAccel.getY();
                changed = true;
            }
        }

        if (!Double.isNaN(maxAccelRotation)) {
            double rotationAccelMagnitude = newSpeeds.omegaRadiansPerSecond
                    - previousSpeeds.omegaRadiansPerSecond;
            if (rotationAccelMagnitude > maxAccelRotation) {
                newSpeeds.omegaRadiansPerSecond = previousSpeeds.omegaRadiansPerSecond
                        + maxAccelRotation;
                changed = true;
            } else if (rotationAccelMagnitude < -maxAccelRotation) {
                newSpeeds.omegaRadiansPerSecond = previousSpeeds.omegaRadiansPerSecond
                        - maxAccelRotation;
                changed = true;
            }
        }

        return changed;
    }
}
