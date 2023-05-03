package org.victorrobotics.frc.dtlib.log.var;

import org.victorrobotics.frc.dtlib.log.DTDataLogVar;

import java.util.function.Supplier;

public class DTDataLogDoubleArray extends DTDataLogVar {
    private final Supplier<double[]> supplier;

    protected DTDataLogDoubleArray(String name, Supplier<double[]> supplier) {
        super(name, 0x00);
        this.supplier = supplier;
    }

    @Override
    protected void writeData() {
        double[] data = supplier.get();
        dataWriter.writeDoubleArray(data);
    }
}
