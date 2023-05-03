package org.victorrobotics.frc.dtlib.log.var;

import org.victorrobotics.frc.dtlib.log.DTDataLogVar;

import java.util.function.Supplier;

public class DTDataLogByteArray extends DTDataLogVar {
    private final Supplier<byte[]> supplier;

    protected DTDataLogByteArray(String name, Supplier<byte[]> supplier) {
        super(name, 0x00);
        this.supplier = supplier;
    }

    @Override
    protected void writeData() {
        byte[] data = supplier.get();
        dataWriter.writeByteArray(data);
    }
}
