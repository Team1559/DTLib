package org.victorrobotics.frc.dtlib.log.var;

import org.victorrobotics.frc.dtlib.log.DTDataLogVar;

import java.util.function.BooleanSupplier;

public class DTDataLogBoolean extends DTDataLogVar {
    private final BooleanSupplier supplier;

    public DTDataLogBoolean(String name, BooleanSupplier supplier) {
        super(name, 0x22);
        this.supplier = supplier;
    }

    @Override
    protected void writeData() {
        dataWriter.writeBoolean(supplier.getAsBoolean());
    }
}
