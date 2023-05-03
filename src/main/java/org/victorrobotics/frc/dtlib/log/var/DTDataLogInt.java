package org.victorrobotics.frc.dtlib.log.var;

import org.victorrobotics.frc.dtlib.log.DTDataLogVar;

import java.util.function.IntSupplier;

public class DTDataLogInt extends DTDataLogVar {
    private final IntSupplier supplier;

    public DTDataLogInt(String name, IntSupplier supplier) {
        super(name, 0x24);
        this.supplier = supplier;
    }

    @Override
    protected void writeData() {
        dataWriter.writeInt(supplier.getAsInt());
    }
}
