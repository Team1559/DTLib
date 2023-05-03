package org.victorrobotics.frc.dtlib.log.var;

import org.victorrobotics.frc.dtlib.log.DTDataLogVar;

import java.util.function.LongSupplier;

public class DTDataLogLong extends DTDataLogVar {
    private final LongSupplier supplier;

    public DTDataLogLong(String name, LongSupplier supplier) {
        super(name, 0x23);
        this.supplier = supplier;
    }

    @Override
    protected void writeData() {
        dataWriter.writeLong(supplier.getAsLong());
    }
}
