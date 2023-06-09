package org.victorrobotics.frc.dtlib.function.supplier;

import java.util.function.LongSupplier;

public interface DTConditionalLongSupplier extends LongSupplier {
    @Override
    long getAsLong();

    boolean update();
}
