package org.victorrobotics.frc.dtlib.log.var;

import org.victorrobotics.frc.dtlib.log.DTDataLogVar;

import java.util.function.Supplier;

public class DTDataLogString extends DTDataLogVar {
    private final Supplier<String> supplier;

    public DTDataLogString(String name, Supplier<String> supplier) {
        super(name, 0x22);
        this.supplier = supplier;
    }

    @Override
    protected void writeData() {
        String str = supplier.get();
        dataWriter.writeStringUTF(str);
    }
}
