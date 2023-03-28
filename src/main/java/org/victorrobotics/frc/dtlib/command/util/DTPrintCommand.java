package org.victorrobotics.frc.dtlib.command.util;

import org.victorrobotics.frc.dtlib.command.test.DTTestCommand;

import edu.wpi.first.wpilibj2.command.PrintCommand;

public class DTPrintCommand extends PrintCommand implements DTTestCommand {
    public DTPrintCommand(String message) {
        super(message);
    }
}
