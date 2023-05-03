package org.victorrobotics.frc.dtlib.log.var;

import org.victorrobotics.frc.dtlib.function.supplier.DTByteSupplier;
import org.victorrobotics.frc.dtlib.log.DTDataLogVar;

public class DTDataLogByte extends DTDataLogVar {
    private final DTByteSupplier supplier;

    public DTDataLogByte(String name, DTByteSupplier supplier) {
        super(name, 0x22);
        this.supplier = supplier;
    }

    @Override
    protected void writeData() {
        dataWriter.writeByte(supplier.getAsByte());
    }
}
