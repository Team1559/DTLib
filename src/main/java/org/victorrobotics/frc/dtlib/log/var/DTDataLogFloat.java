package org.victorrobotics.frc.dtlib.log.var;

import org.victorrobotics.frc.dtlib.log.DTDataLogVar;

import edu.wpi.first.util.function.FloatSupplier;

public class DTDataLogFloat extends DTDataLogVar {
    private final FloatSupplier supplier;

    public DTDataLogFloat(String name, FloatSupplier supplier) {
        super(name, 0x22);
        this.supplier = supplier;
    }

    @Override
    protected void writeData() {
        dataWriter.writeFloat(supplier.getAsFloat());
    }
}
