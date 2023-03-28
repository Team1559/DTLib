package org.victorrobotics.frc.dtlib.command.test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DTSequentialTestCommandGroup extends DTTestCommandGroupBase<SequentialCommandGroup> {
    public DTSequentialTestCommandGroup(DTTestCommand[] commands) {
        super(SequentialCommandGroup::new, commands);
    }
}
