package org.victorrobotics.dtlib.drivetrain;

import org.victorrobotics.dtlib.DTRobot;
import org.victorrobotics.dtlib.math.geometry.DTVector2dR;

public final class DTAccelerationLimit {
  // These are in standard units
  public final double maxTranslation;
  public final double maxRotation;

  // These are in meters per second per robot cycle
  private final double maxTranslationPerCycle;
  private final double maxRotationPerCycle;

  public DTAccelerationLimit() {
    maxTranslation = Double.NaN;
    maxRotation = Double.NaN;
    maxTranslationPerCycle = Double.NaN;
    maxRotationPerCycle = Double.NaN;
  }

  public DTAccelerationLimit(double translation, double rotation) {
    maxTranslation = Double.isFinite(translation) ? Math.abs(translation) : Double.NaN;
    maxRotation = Double.isFinite(rotation) ? Math.abs(rotation) : Double.NaN;
    maxTranslationPerCycle = maxTranslation * DTRobot.PERIOD_SECONDS;
    maxRotationPerCycle = maxRotation * DTRobot.PERIOD_SECONDS;
  }

  public boolean apply(DTVector2dR newSpeeds, DTVector2dR previousSpeeds) {
    boolean change = false;

    DTVector2dR cycleAcceleration = newSpeeds.clone()
                                             .subtract(previousSpeeds);

    double translationAccel = cycleAcceleration.getNorm();
    if (!Double.isNaN(maxTranslationPerCycle) && translationAccel > maxTranslationPerCycle) {
      cycleAcceleration.multiply(maxTranslationPerCycle / translationAccel);
      change = true;
    }

    double rotationAccel = Math.abs(cycleAcceleration.getR());
    if (!Double.isNaN(maxRotationPerCycle) && rotationAccel > maxRotationPerCycle) {
      cycleAcceleration.multiply(maxRotationPerCycle / rotationAccel);
      change = true;
    }

    if (change) {
      newSpeeds.set(previousSpeeds.clone()
                                  .add(cycleAcceleration.normalize(maxTranslationPerCycle)));
    }

    return change;
  }
}
