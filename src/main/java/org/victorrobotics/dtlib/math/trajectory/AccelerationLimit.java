package org.victorrobotics.dtlib.math.trajectory;

import org.victorrobotics.dtlib.DTRobot;
import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

public final class AccelerationLimit {
  // These are in standard units
  public final double maxTranslation;
  public final double maxRotation;

  // These are in meters per second per robot cycle
  private final double maxTranslationPerCycle;
  private final double maxRotationPerCycle;

  public AccelerationLimit() {
    maxTranslation = Double.NaN;
    maxRotation = Double.NaN;
    maxTranslationPerCycle = Double.NaN;
    maxRotationPerCycle = Double.NaN;
  }

  public AccelerationLimit(double translation, double rotation) {
    maxTranslation = Double.isFinite(translation) ? Math.abs(translation) : Double.NaN;
    maxRotation = Double.isFinite(rotation) ? Math.abs(rotation) : Double.NaN;
    maxTranslationPerCycle = maxTranslation * DTRobot.PERIOD_SECONDS;
    maxRotationPerCycle = maxRotation * DTRobot.PERIOD_SECONDS;
  }

  public boolean apply(Vector2D_R newSpeeds, Vector2D_R previousSpeeds) {
    boolean change = false;

    Vector2D_R cycleAcceleration = newSpeeds.clone()
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
