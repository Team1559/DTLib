package org.victorrobotics.dtlib.function.supplier;

import java.util.function.IntSupplier;

public interface DTConditionalIntSupplier extends IntSupplier {
  @Override
  int getAsInt();

  boolean update();
}
