package org.victorrobotics.frc.dtlib.command.test;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class DTParallelTestCommandGroup extends DTTestCommandGroupBase<ParallelCommandGroup> {
    public DTParallelTestCommandGroup(DTTestCommand[] commands) {
        super(ParallelCommandGroup::new, commands);
    }
}
