package org.victorrobotics.frc.dtlib.log.var;

import org.victorrobotics.frc.dtlib.log.DTDataLogVar;

import java.util.function.Supplier;

public class DTDataLogFloatArray extends DTDataLogVar {
    private final Supplier<float[]> supplier;

    protected DTDataLogFloatArray(String name, Supplier<float[]> supplier) {
        super(name, 0x00);
        this.supplier = supplier;
    }

    @Override
    protected void writeData() {
        float[] data = supplier.get();
        dataWriter.writeFloatArray(data);
    }
}
