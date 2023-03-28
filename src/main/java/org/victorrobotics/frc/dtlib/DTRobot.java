package org.victorrobotics.frc.dtlib;

import org.victorrobotics.frc.dtlib.command.test.DTSelfTestCommand;
import org.victorrobotics.frc.dtlib.command.util.DTPrintCommand;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public abstract class DTRobot extends TimedRobot {
    private final List<DTSubsystem> subsystems;
    private Compressor              pneumaticsCompressor;
    private boolean                 compressorEnabled;

    private Command autonomousCommand;

    protected DTRobot() {
        super(0.02);
        subsystems = new ArrayList<>();
        pneumaticsCompressor = null;
        compressorEnabled = true;
    }

    protected abstract void onBootUp();

    protected abstract void bindCommands();

    protected abstract Command getAutonomousCommand();

    public final DTSelfTestCommand getSelfTestCommand() {
        return new DTSelfTestCommand(subsystems.toArray(DTSubsystem[]::new));
    }

    protected final void registerSubsystem(DTSubsystem subsystem) {
        subsystems.add(subsystem);
    }

    protected final void setCompressor(Compressor compressor) {
        if (pneumaticsCompressor != null) {
            pneumaticsCompressor.disable();
        }
        pneumaticsCompressor = compressor;
    }

    private void disableCompressor() {
        if (pneumaticsCompressor != null) {
            pneumaticsCompressor.disable();
        }
    }

    private void enableCompressor() {
        if (pneumaticsCompressor != null && compressorEnabled) {
            pneumaticsCompressor.enableDigital();
        }
    }

    protected void setCompressorControl(boolean enable) {
        compressorEnabled = enable;
        if (!enable && pneumaticsCompressor != null) {
            pneumaticsCompressor.disable();
        }
    }

    @Override
    public final void robotInit() {
        try {
            onBootUp();
            bindCommands();
        } catch (Exception e) {
            // TODO: what to do here?
        }
    }

    @Override
    public final void robotPeriodic() {
        getCommandScheduler().run();
    }

    @Override
    public final void simulationInit() {}

    @Override
    public final void simulationPeriodic() {}

    @Override
    public final void autonomousInit() {
        enableCompressor();
        try {
            autonomousCommand = getAutonomousCommand();
        } catch (Exception e) {
            autonomousCommand = new DTPrintCommand("Failed to fetch autonomous command");
        }
        CommandScheduler.getInstance()
                        .schedule(autonomousCommand);
    }

    @Override
    public final void autonomousPeriodic() {}

    @Override
    public final void autonomousExit() {
        autonomousCommand.cancel();
    }

    @Override
    public final void teleopInit() {
        enableCompressor();
    }

    @Override
    public final void teleopPeriodic() {}

    @Override
    public final void teleopExit() {}

    @Override
    public final void testInit() {
        enableCompressor();
    }

    @Override
    public final void testPeriodic() {}

    @Override
    public final void testExit() {}

    @Override
    public final void disabledInit() {
        disableCompressor();
    }

    @Override
    public final void disabledPeriodic() {}

    @Override
    public final void disabledExit() {}

    public static CommandScheduler getCommandScheduler() {
        return CommandScheduler.getInstance();
    }
}
