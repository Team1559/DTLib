package org.victorrobotics.frc.dtlib.drivetrain;

import org.victorrobotics.frc.dtlib.math.geometry.DTVector2DR;

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

        DTVector2DR oldVelocityVec = new DTVector2DR(previousSpeeds);
        DTVector2DR cycleAccelVec = new DTVector2DR(newSpeeds).subtract(oldVelocityVec);
        double cycleAccelTranslation = cycleAccelVec.hypotenuse();
        if (cycleAccelTranslation > maxAccelTranslationPerCycle) {
            cycleAccelVec.multiply(maxAccelTranslationPerCycle / cycleAccelTranslation);
            newSpeeds.vxMetersPerSecond = previousSpeeds.vxMetersPerSecond + cycleAccelVec.getX();
            newSpeeds.vyMetersPerSecond = previousSpeeds.vyMetersPerSecond + cycleAccelVec.getY();
            changed = true;
        }

        double cycleAccelRotation = newSpeeds.omegaRadiansPerSecond - previousSpeeds.omegaRadiansPerSecond;
        if (cycleAccelRotation > maxAccelRotationPerCycle) {
            newSpeeds.omegaRadiansPerSecond = previousSpeeds.omegaRadiansPerSecond + maxAccelRotationPerCycle;
            changed = true;
        } else if (cycleAccelRotation < -maxAccelRotationPerCycle) {
            newSpeeds.omegaRadiansPerSecond = previousSpeeds.omegaRadiansPerSecond - maxAccelRotationPerCycle;
            changed = true;
        }

        return changed;
    }
}
