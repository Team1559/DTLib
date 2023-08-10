package org.victorrobotics.frc.dtlib.log;

public abstract class DTDataLogVar {
    protected static final DTDataLogWriter dataWriter = DTDataLogger.dataWriter;

    private final String name;
    private final int    typeByte;

    private int handle;

    protected DTDataLogVar(String name, int typeByte) {
        this.name = name;
        this.typeByte = typeByte & 0xFF;
    }

    protected abstract void writeData();

    final void log() {
        if (handle == 0) {
            // Declare new variable
            handle = DTDataLogger.generateHandle();
            dataWriter.writeShort(typeByte);

        }
        // Encoding is infered from handle
        dataWriter.writeShort(handle);
        writeData();
    }
}
