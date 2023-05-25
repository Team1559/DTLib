package org.victorrobotics.frc.dtlib.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class DTAccelerationLimit {
    public static final double DEFAULT_MAX_TRANSLATION = 250;
    public static final double DEFAULT_MAX_ROTATION    = 500;
    public static final double DEFAULT_CYCLE_LENGTH    = 0.02;

    // These are in standard units
    public final double maxAccelTranslation;
    public final double maxAccelRotation;

    // These are in meters per second per robot cycle
    private final double maxAccelTranslationPerCycle;
    private final double maxAccelRotationPerCycle;

    public DTAccelerationLimit() {
        this(Double.NaN, Double.NaN, Double.NaN);
    }

    public DTAccelerationLimit(double translation, double rotation, double cycleLength) {
        if (!Double.isFinite(cycleLength)) {
            cycleLength = DEFAULT_CYCLE_LENGTH;
        } else {
            cycleLength = Math.abs(cycleLength);
        }

        maxAccelTranslation = Double.isFinite(translation) ? Math.abs(translation) : DEFAULT_MAX_TRANSLATION;
        maxAccelRotation = Double.isFinite(rotation) ? Math.abs(rotation) : DEFAULT_MAX_ROTATION;
        maxAccelTranslationPerCycle = maxAccelTranslation * cycleLength;
        maxAccelRotationPerCycle = maxAccelRotation * cycleLength;
    }

    public boolean apply(ChassisSpeeds newSpeeds, ChassisSpeeds previousSpeeds) {
        boolean changed = false;

        Translation2d oldTranslation = new Translation2d(previousSpeeds.vxMetersPerSecond,
                previousSpeeds.vyMetersPerSecond);
        Translation2d newTranslation = new Translation2d(newSpeeds.vxMetersPerSecond, newSpeeds.vyMetersPerSecond);
        Translation2d translationAccel = newTranslation.minus(oldTranslation);
        double translationAccelMagnitude = translationAccel.getNorm();
        if (translationAccelMagnitude > maxAccelTranslationPerCycle) {
            translationAccel = translationAccel.times(maxAccelTranslationPerCycle / translationAccelMagnitude);
            newSpeeds.vxMetersPerSecond = previousSpeeds.vxMetersPerSecond + translationAccel.getX();
            newSpeeds.vyMetersPerSecond = previousSpeeds.vyMetersPerSecond + translationAccel.getY();
            changed = true;
        }

        double rotationAccelMagnitude = newSpeeds.omegaRadiansPerSecond - previousSpeeds.omegaRadiansPerSecond;
        if (rotationAccelMagnitude > maxAccelRotationPerCycle) {
            newSpeeds.omegaRadiansPerSecond = previousSpeeds.omegaRadiansPerSecond + maxAccelRotationPerCycle;
            changed = true;
        } else if (rotationAccelMagnitude < -maxAccelRotationPerCycle) {
            newSpeeds.omegaRadiansPerSecond = previousSpeeds.omegaRadiansPerSecond - maxAccelRotationPerCycle;
            changed = true;
        }

        return changed;
    }
}
