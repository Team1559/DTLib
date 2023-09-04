package org.victorrobotics.frc.dtlib.drivetrain;

import org.victorrobotics.frc.dtlib.exception.DTIllegalArgumentException;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class DTAccelerationLimit {
  // These are in standard units
  public final double maxAccelTranslation;
  public final double maxAccelRotation;

  // These are in meters per second per robot cycle
  private final double maxAccelTranslationPerCycle;
  private final double maxAccelRotationPerCycle;

  public DTAccelerationLimit() {
    maxAccelTranslation = Double.NaN;
    maxAccelRotation = Double.NaN;
    maxAccelTranslationPerCycle = Double.NaN;
    maxAccelRotationPerCycle = Double.NaN;
  }

  public DTAccelerationLimit(double translation, double rotation, double cycleLength) {
    if (!Double.isFinite(cycleLength)) {
      throw new DTIllegalArgumentException("cycleLength must be finite", cycleLength);
    }
    cycleLength = Math.abs(cycleLength);

    maxAccelTranslation = Double.isFinite(translation) ? Math.abs(translation) : Double.NaN;
    maxAccelRotation = Double.isFinite(rotation) ? Math.abs(rotation) : Double.NaN;
    maxAccelTranslationPerCycle = maxAccelTranslation * cycleLength;
    maxAccelRotationPerCycle = maxAccelRotation * cycleLength;
  }

  public boolean apply(ChassisSpeeds newSpeeds, ChassisSpeeds previousSpeeds) {
    boolean changed = false;

    if (!Double.isNaN(maxAccelTranslationPerCycle)) {
      Translation2d oldTranslation = new Translation2d(previousSpeeds.vxMetersPerSecond,
          previousSpeeds.vyMetersPerSecond);
      Translation2d newTranslation = new Translation2d(newSpeeds.vxMetersPerSecond,
          newSpeeds.vyMetersPerSecond);
      Translation2d translationAccel = newTranslation.minus(oldTranslation);
      double translationAccelMagnitude = translationAccel.getNorm();
      if (translationAccelMagnitude > maxAccelTranslationPerCycle) {
        translationAccel = translationAccel.times(
            maxAccelTranslationPerCycle / translationAccelMagnitude);
        newSpeeds.vxMetersPerSecond = previousSpeeds.vxMetersPerSecond + translationAccel.getX();
        newSpeeds.vyMetersPerSecond = previousSpeeds.vyMetersPerSecond + translationAccel.getY();
        changed = true;
      }
    }

    if (!Double.isNaN(maxAccelRotationPerCycle)) {
      double rotationAccelMagnitude = newSpeeds.omegaRadiansPerSecond
          - previousSpeeds.omegaRadiansPerSecond;
      if (rotationAccelMagnitude > maxAccelRotationPerCycle) {
        newSpeeds.omegaRadiansPerSecond = previousSpeeds.omegaRadiansPerSecond
            + maxAccelRotationPerCycle;
        changed = true;
      } else if (rotationAccelMagnitude < -maxAccelRotationPerCycle) {
        newSpeeds.omegaRadiansPerSecond = previousSpeeds.omegaRadiansPerSecond
            - maxAccelRotationPerCycle;
        changed = true;
      }
    }

    return changed;
  }
}
