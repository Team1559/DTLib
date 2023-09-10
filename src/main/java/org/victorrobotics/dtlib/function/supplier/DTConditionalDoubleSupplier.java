package org.victorrobotics.frc.dtlib.function.supplier;

import java.util.function.DoubleSupplier;

public interface DTConditionalDoubleSupplier extends DoubleSupplier {
  @Override
  double getAsDouble();

  boolean update();
}
