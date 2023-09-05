package org.victorrobotics.frc.dtlib;

import org.victorrobotics.frc.dtlib.command.DTCommand;
import org.victorrobotics.frc.dtlib.command.DTCommandScheduler;
import org.victorrobotics.frc.dtlib.log.DTLogRootNode;
import org.victorrobotics.frc.dtlib.log.DTLogger;

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

public abstract class DTRobot {
  public static final double PERIOD_SECONDS = 0.02;
  public static final long   PERIOD_MICROS  = (long) (PERIOD_SECONDS * 1e6);

  private final int notifierHandle;

  private final DSControlWord modeSupplier;
  private final Watchdog      loopOverrun;

  private final List<DTSubsystem> subsystems;

  private final DTLogRootNode logRoot;

  private Compressor compressor;
  private boolean    compressorEnabled;

  private Mode currentMode;
  private Mode previousMode;

  private DTCommand autoCommand;

  protected DTRobot() {
    notifierHandle = NotifierJNI.initializeNotifier();
    NotifierJNI.setNotifierName(notifierHandle, "DTRobot");

    modeSupplier = new DSControlWord();
    currentMode = Mode.DISABLED;
    previousMode = Mode.DISABLED;

    loopOverrun = new Watchdog(PERIOD_SECONDS, () -> {});
    loopOverrun.suppressTimeoutMessage(true);

    subsystems = new ArrayList<>();
    compressor = null;
    compressorEnabled = true;

    logRoot = new DTLogRootNode(this);
  }

  /**
   * When the robot boots up, this method will be executed to construct and initialize any robot
   * hardware, subsystems, etc.
   */
  protected abstract void init();

  /**
   * If the robot is a simulation, this method is where additional setup logic will be executed
   */
  protected abstract void simulationInit();

  /**
   * This method will be called after initialization to bind any commands to triggers such as
   * controller buttons
   */
  protected abstract void bindCommands();

  /**
   * This method will be called every robot cycle before commands are executed, and can be used for
   * purposes such as logging values to the console
   */
  protected abstract void periodic();

  /**
   * @return the user-supplied command to be executed when autonomous mode is enabled
   */
  protected abstract DTCommand getAutoCommand();

  /**
   * Start the robot program. Replaces {@link RobotBase#runRobot RobotBase::runRobot()}.
   */
  public final void start() {
    String className = getClass().getSimpleName();
    System.out.println("[DTLib] " + className + " initializing...");
    // Do not catch init exceptions
    init();
    if (isSimulation()) {
      simulationInit();
    }
    bindCommands();

    while (!DTLogger.init()) {
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        // ignore
      }
    }

    DriverStationJNI.observeUserProgramStarting();
    System.out.println("[DTLib] " + className + " initialization complete");

    long triggerTime = 0;
    while (true) {
      // Wait to be woken up
      NotifierJNI.updateNotifierAlarm(notifierHandle, triggerTime);
      long time = NotifierJNI.waitForNotifierAlarm(notifierHandle);
      if (time == 0) {
        // Notifier has been stopped, exit
        break;
      }
      // Compute next cycle start time
      triggerTime += PERIOD_MICROS;

      // Load new input data
      loopOverrun.enable();
      DriverStation.refreshData();
      refreshMode();

      // Execute code for this cycle
      runModeChange();
      runPeriodic();
      runCurrentMode();
      log();

      loopOverrun.disable();

      if (loopOverrun.isExpired()) {
        DriverStation.reportWarning("Loop time of " + PERIOD_SECONDS + "s overrun by "
            + (loopOverrun.getTime() - PERIOD_SECONDS) + "s", false);
        loopOverrun.printEpochs();
      }
    }
  }

  public final void end() {
    NotifierJNI.stopNotifier(notifierHandle);
  }

  private void refreshMode() {
    modeSupplier.refresh();
    previousMode = currentMode;

    if (modeSupplier.isEStopped()) {
      currentMode = Mode.E_STOP;
    } else if (!modeSupplier.isEnabled()) {
      currentMode = Mode.DISABLED;
    } else if (modeSupplier.isAutonomous()) {
      currentMode = Mode.AUTO;
    } else if (modeSupplier.isTest()) {
      currentMode = Mode.TEST;
    } else {
      currentMode = Mode.TELEOP;
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
      DTCommandScheduler.schedule(autoCommand);
    } else if (previousMode == Mode.AUTO) {
      DTCommandScheduler.cancel(autoCommand);
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
      case E_STOP:
        DriverStationJNI.observeUserProgramDisabled();
        break;
      case AUTO:
        DriverStationJNI.observeUserProgramAutonomous();
        break;
      case TELEOP:
        DriverStationJNI.observeUserProgramTeleop();
        break;
      case TEST:
        DriverStationJNI.observeUserProgramTest();
        break;
    }
    DTCommandScheduler.run();
    loopOverrun.addEpoch("Execute " + currentMode);
  }

  private void log() {
    DTLogger.logNewTimestamp();
    if (currentMode != previousMode) {
      switch (currentMode) {
        case DISABLED:
        case E_STOP:
          DTLogger.getWriter()
                  .writeShort(0x04);
          break;
        case AUTO:
          DTLogger.getWriter()
                  .writeShort(0x05);
          break;
        case TELEOP:
          DTLogger.getWriter()
                  .writeShort(0x06);
          break;
        case TEST:
          DTLogger.getWriter()
                  .writeShort(0x07);
          break;
      }
    }
    logRoot.log();
    DTLogger.flush();
    loopOverrun.addEpoch("DTLog");
  }

  public final Mode getCurrentMode() {
    return currentMode;
  }

  public static boolean isSimulation() {
    return RuntimeType.getValue(HALUtil.getHALRuntimeType()) == RuntimeType.kSimulation;
  }

  public static boolean isReal() {
    return !isSimulation();
  }

  public Alliance getAlliance() {
    return Alliance.fromDS(DriverStationJNI.getAllianceStation());
  }

  protected final void registerSubsystem(DTSubsystem subsystem) {
    subsystems.add(subsystem);
  }

  protected abstract DTCommand getSelfTestCommand();

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
