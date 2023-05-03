package org.victorrobotics.frc.dtlib.log.var;

import org.victorrobotics.frc.dtlib.log.DTDataLogVar;

import java.util.function.Supplier;

public class DTDataLogBooleanArray extends DTDataLogVar {
    private final Supplier<boolean[]> supplier;

    public DTDataLogBooleanArray(String name, Supplier<boolean[]> supplier) {
        super(name, 0x00);
        this.supplier = supplier;
    }

    @Override
    protected void writeData() {
        boolean[] data = supplier.get();
        dataWriter.writeBooleanArray(data);
    }
}
