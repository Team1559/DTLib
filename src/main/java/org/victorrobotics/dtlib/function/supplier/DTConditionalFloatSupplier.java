package org.victorrobotics.dtlib.function.supplier;

import edu.wpi.first.util.function.FloatSupplier;

public interface DTConditionalFloatSupplier extends FloatSupplier {
  @Override
  float getAsFloat();

  boolean update();
}
