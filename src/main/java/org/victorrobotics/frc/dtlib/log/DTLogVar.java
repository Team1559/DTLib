package org.victorrobotics.frc.dtlib.log;

public abstract class DTLogVar {
    protected static final DTLogWriter dataWriter = DTLogger.dataWriter;

    private final String name;
    private final int    typeByte;

    private int handle;

    protected DTLogVar(String name, int typeByte) {
        this.name = name;
        this.typeByte = typeByte & 0xFF;
    }

    protected abstract void writeData();

    final void log() {
        if (handle == 0) {
            // Declare new variable
            handle = DTLogger.generateHandle();
            dataWriter.writeShort(typeByte);

        }
        // Encoding is infered from handle
        dataWriter.writeShort(handle);
        writeData();
    }
}
