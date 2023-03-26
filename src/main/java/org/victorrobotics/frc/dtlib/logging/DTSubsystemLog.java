package org.victorrobotics.frc.dtlib.logging;

import org.victorrobotics.frc.dtlib.DTSubsystem;

import java.io.DataOutputStream;

import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;

public class DTSubsystemLog {
    private final DTLog       parent;
    private final DTSubsystem subsystem;

    public DTSubsystemLog(DTLog parent, DTSubsystem subsystem) {
        this.parent = parent;
        this.subsystem = subsystem;
    }

    public void writeFormat(DataOutputStream dataStream) {
        
    }
}
