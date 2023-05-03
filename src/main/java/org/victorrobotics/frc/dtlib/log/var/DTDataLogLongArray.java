package org.victorrobotics.frc.dtlib.log.var;

import org.victorrobotics.frc.dtlib.log.DTDataLogVar;

import java.util.function.Supplier;

public class DTDataLogLongArray extends DTDataLogVar {
    private final Supplier<long[]> supplier;

    protected DTDataLogLongArray(String name, Supplier<long[]> supplier) {
        super(name, 0x00);
        this.supplier = supplier;
    }

    @Override
    protected void writeData() {
        long[] data = supplier.get();
        dataWriter.writeLongArray(data);
    }
}
