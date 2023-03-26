package org.victorrobotics.frc.dtlib.function.supplier;

import java.util.function.BooleanSupplier;

public interface DTConditionalBooleanSupplier extends BooleanSupplier {
    @Override
    boolean getAsBoolean();

    boolean update();
}
