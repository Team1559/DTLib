package org.victorrobotics.frc.dtlib;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public abstract class DTRobot extends TimedRobot {
    private final Set<DTSubsystem> subsystems;

    protected DTRobot() {
        super(0.02);
        subsystems = new HashSet<>();
    }

    @Override
    public void robotPeriodic() {
        
    }

    protected final void registerSubsystem(DTSubsystem subsystem) {
        subsystems.add(subsystem);
    }

    public static CommandScheduler getCommandScheduler() {
        return CommandScheduler.getInstance();
    }
}
