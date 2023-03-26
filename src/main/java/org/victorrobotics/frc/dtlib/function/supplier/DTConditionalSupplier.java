package org.victorrobotics.frc.dtlib.function.supplier;

import java.util.function.Supplier;

public interface DTConditionalSupplier<T> extends Supplier<T> {
    @Override
    T get();

    boolean update();
}
