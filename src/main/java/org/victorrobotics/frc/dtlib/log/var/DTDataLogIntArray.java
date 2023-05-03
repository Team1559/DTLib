package org.victorrobotics.frc.dtlib.log.var;

import org.victorrobotics.frc.dtlib.log.DTDataLogVar;

import java.util.function.Supplier;

public class DTDataLogIntArray extends DTDataLogVar {
    private final Supplier<int[]> supplier;

    protected DTDataLogIntArray(String name, Supplier<int[]> supplier) {
        super(name, 0x00);
        this.supplier = supplier;
    }

    @Override
    protected void writeData() {
        int[] data = supplier.get();
        dataWriter.writeIntArray(data);
    }
}
