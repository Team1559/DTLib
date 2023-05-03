package org.victorrobotics.frc.dtlib.log.var;

import org.victorrobotics.frc.dtlib.log.DTDataLogVar;

import java.util.function.DoubleSupplier;

public class DTDataLogDouble extends DTDataLogVar {
    private final DoubleSupplier supplier;

    public DTDataLogDouble(String name, DoubleSupplier supplier) {
        super(name, 0x22);
        this.supplier = supplier;
    }

    @Override
    protected void writeData() {
        dataWriter.writeDouble(supplier.getAsDouble());
    }
}
