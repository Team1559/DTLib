package org.victorrobotics.frc.dtlib;

import org.victorrobotics.frc.dtlib.command.DTCommand;
import org.victorrobotics.frc.dtlib.controller.DTController;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DSControlWord;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.ejml.simple.UnsupportedOperation;

public abstract class DTRobot {
    public static final double PERIOD_SECONDS = 0.02;
    public static final long   PERIOD_MICROS  = (long) (PERIOD_SECONDS * 1e6);

    private final int notifierHandle;

    private final DSControlWord modeSupplier;
    private volatile boolean    running;
    private final Watchdog      loopOverrun;

    private final List<DTSubsystem> subsystems;

    private Compressor compressor;
    private boolean    compressorEnabled;

    private Mode currentMode;
    private Mode previousMode;

    private Command autoCommand;

    protected DTRobot() {
        notifierHandle = NotifierJNI.initializeNotifier();
        NotifierJNI.setNotifierName(notifierHandle, "DTRobot");

        modeSupplier = new DSControlWord();
        currentMode = Mode.DISABLED;
        previousMode = Mode.DISABLED;

        loopOverrun = new Watchdog(PERIOD_MICROS * 1e-6, () -> {
        });
        loopOverrun.suppressTimeoutMessage(true);

        subsystems = new ArrayList<>();
        compressor = null;
        compressorEnabled = true;
    }

    /**
     * When the robot boots up, this method will be executed to construct and
     * initialize any robot hardware, subsystems, etc.
     */
    protected abstract void init();

    /**
     * If the robot is a simulation, this method is where additional setup logic
     * will be executed
     */
    protected abstract void simulationInit();

    /**
     * This method will be called after initialization to bind any commands to
     * triggers such as controller buttons
     */
    protected abstract void bindCommands();

    /**
     * This method will be called every robot cycle before commands are
     * executed, and can be used for purposes such as logging values to the
     * console
     */
    protected abstract void periodic();

    /**
     * @return the user-supplied command to be executed when autonomous mode is
     *         enabled
     */
    protected abstract Command getAutoCommand();

    /**
     * Start the robot program. Replaces {@link RobotBase#runRobot
     * RobotBase::runRobot()}.
     */
    public final void start() {
        running = true;
        String className = getClass().getSimpleName();
        System.out.println("[DTLib] " + className + " initializing...");
        // Do not catch init exceptions
        init();
        if (isSimulation()) {
            simulationInit();
        }
        bindCommands();

        // No exceptions? Robot is ready to be enabled
        DriverStationJNI.observeUserProgramStarting();
        System.out.println("[DTLib] " + className + " initialization complete");

        long triggerTime = 0;
        while (running) {
            // Wait to be woken up
            NotifierJNI.updateNotifierAlarm(notifierHandle, triggerTime);
            long time = NotifierJNI.waitForNotifierAlarm(notifierHandle);
            if (time == 0) {
                // TODO: see what 0 represents
                break;
            }
            // Compute next cycle start time
            loopOverrun.reset();
            if (time - triggerTime >= 1000) {
                // If more than 1ms slow, prevent snowballing
                triggerTime = time;
            }
            triggerTime += PERIOD_MICROS;

            // Load new input data
            DriverStation.refreshData();
            DTController.refreshAll();
            refreshMode();

            // Execute code for this cycle
            runPeriodic();
            runModeChange();
            runCurrentMode();

            loopOverrun.disable();

            if (loopOverrun.isExpired()) {
                DriverStation.reportWarning("Loop time of " + PERIOD_SECONDS + "s overrun by "
                        + (loopOverrun.getTime() - PERIOD_SECONDS) + "s", false);
                loopOverrun.printEpochs();
            }
        }
    }

    public final void end() {
        running = false;
        NotifierJNI.stopNotifier(notifierHandle);
    }

    private void refreshMode() {
        modeSupplier.refresh();
        previousMode = currentMode;

        if (modeSupplier.isEStopped()) {
            currentMode = Mode.E_STOP;
        } else if (modeSupplier.isDisabled()) {
            currentMode = Mode.DISABLED;
        } else if (modeSupplier.isAutonomous()) {
            currentMode = Mode.AUTO;
        } else if (modeSupplier.isTest()) {
            currentMode = Mode.TEST;
        } else if (modeSupplier.isTeleop()) {
            currentMode = Mode.TELEOP;
        } else {
            // should never occur, just for safety
            currentMode = Mode.DISABLED;
        }
        loopOverrun.addEpoch("DS refresh");
    }

    private void runPeriodic() {
        periodic();
        loopOverrun.addEpoch("Periodic");
    }

    private void runModeChange() {
        if (currentMode == previousMode) {
            return;
        }

        if (currentMode == Mode.AUTO) {
            autoCommand = getAutoCommand();
            commandScheduler().schedule(autoCommand);
        } else if (previousMode == Mode.AUTO) {
            commandScheduler().cancel(autoCommand);
        }

        if (currentMode.isEnabled && !previousMode.isEnabled) {
            enableCompressor();
        } else if (!currentMode.isEnabled && previousMode.isEnabled) {
            disableCompressor();
        }

        loopOverrun.addEpoch("Mode change");
    }

    private void runCurrentMode() {
        switch (currentMode) {
            case DISABLED:
                DriverStationJNI.observeUserProgramDisabled();
                break;
            case E_STOP:
                DriverStationJNI.observeUserProgramDisabled();
                break;
            case AUTO:
                DriverStationJNI.observeUserProgramAutonomous();
                commandScheduler().run();
                break;
            case TELEOP:
                DriverStationJNI.observeUserProgramTeleop();
                commandScheduler().run();
                break;
            case TEST:
                DriverStationJNI.observeUserProgramTest();
                break;
        }
        loopOverrun.addEpoch("Execute " + currentMode);
    }

    public final Mode getCurrentMode() {
        return currentMode;
    }

    public boolean isSimulation() {
        return RuntimeType.getValue(HALUtil.getHALRuntimeType()) == RuntimeType.kSimulation;
    }

    public boolean isReal() {
        return !isSimulation();
    }

    public Alliance getAlliance() {
        return Alliance.fromDS(DriverStationJNI.getAllianceStation());
    }

    protected final void registerSubsystem(DTSubsystem subsystem) {
        subsystems.add(subsystem);
    }

    private final DTCommand getSelfTestCommand() {
        throw new UnsupportedOperation();
    }

    protected final void configCompressor(int module, PneumaticsModuleType type) {
        if (compressor != null) {
            compressor.disable();
        }
        compressor = new Compressor(module, type);
    }

    protected void configCompressorControl(boolean enable) {
        compressorEnabled = enable;
        if (!enable && compressor != null) {
            compressor.disable();
        }
    }

    private void enableCompressor() {
        if (compressor != null && compressorEnabled) {
            compressor.enableDigital();
        }
    }

    private void disableCompressor() {
        if (compressor != null) {
            compressor.disable();
        }
    }

    protected static CommandScheduler commandScheduler() {
        return CommandScheduler.getInstance();
    }

    public enum Mode {
        DISABLED(false),
        E_STOP(false),
        AUTO(true),
        TELEOP(true),
        TEST(true);

        public final boolean isEnabled;

        Mode(boolean isEnabled) {
            this.isEnabled = isEnabled;
        }
    }

    public enum Alliance {
        RED_1(1, true),
        RED_2(2, true),
        RED_3(3, true),
        BLUE_1(1, false),
        BLUE_2(2, false),
        BLUE_3(3, false);

        public final int     id;
        public final boolean isRed;

        Alliance(int id, boolean isRed) {
            this.id = id;
            this.isRed = isRed;
        }

        public static Alliance fromDS(AllianceStationID a) {
            return Alliance.values()[a.ordinal()];
        }
    }
}
