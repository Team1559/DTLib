package org.victorrobotics.frc.dtlib.logging;

public interface DTLoggable {
    default String getTypeID() {
        return getClass().getSimpleName();
    }

    void encode();
}
