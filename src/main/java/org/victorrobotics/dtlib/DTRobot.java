package org.victorrobotics.dtlib;

import org.victorrobotics.dtlib.command.DTCommand;
import org.victorrobotics.dtlib.command.DTCommandScheduler;
import org.victorrobotics.dtlib.log.DTLogRootNode;
import org.victorrobotics.dtlib.log.DTLogger;
import org.victorrobotics.dtlib.log.DTWatchdog;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DSControlWord;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RuntimeType;

public abstract class DTRobot {
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

  public enum AllianceStation {
    RED_1(1, true),
    RED_2(2, true),
    RED_3(3, true),
    BLUE_1(1, false),
    BLUE_2(2, false),
    BLUE_3(3, false);

    public final int     station;
    public final boolean isRed;

    AllianceStation(int station, boolean isRed) {
      this.station = station;
      this.isRed = isRed;
    }

    static AllianceStation fromDS(AllianceStationID id) {
      return AllianceStation.values()[id.ordinal()];
    }
  }

  public static final double PERIOD_SECONDS = 0.02;
  public static final long   PERIOD_MICROS  = (long) (PERIOD_SECONDS * 1e6);

  private static final DSControlWord CONTROL_WORD  = new DSControlWord();
  private static final boolean       IS_SIMULATION =
      RuntimeType.getValue(HALUtil.getHALRuntimeType()) == RuntimeType.kSimulation;
  private static final AtomicBoolean RUN           = new AtomicBoolean();

  private static Mode currentMode  = Mode.DISABLED;
  private static Mode previousMode = Mode.DISABLED;

  private static AllianceStation alliance;

  private final DTLogRootNode logRoot;

  private Compressor compressor;

  private DTCommand autoCommand;

  protected DTRobot() {
    this.logRoot = new DTLogRootNode(this);
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
   * This method will be called every robot cycle before commands are executed,
   * and can be used for purposes such as logging values to the console
   */
  protected abstract void periodic();

  /**
   * @return the user-supplied command to be executed when autonomous mode is
   *         enabled
   */
  protected abstract DTCommand getAutoCommand();

  /**
   * @return the user-supplied command to execute when the robot self-tests
   */
  protected abstract DTCommand getSelfTestCommand();

  public String getName() {
    return getClass().getSimpleName();
  }

  private void runModeChange() {
    if (currentMode == previousMode) return;

    if (currentMode == Mode.AUTO) {
      DTWatchdog.startEpoch();
      autoCommand = getAutoCommand();
      DTWatchdog.addEpoch("getAutoCommand()");
      DTCommandScheduler.schedule(autoCommand);
    } else if (previousMode == Mode.AUTO) {
      DTCommandScheduler.cancel(autoCommand);
    }

    if (compressor != null) {
      if (currentMode.isEnabled && !previousMode.isEnabled) {
        compressor.enableDigital();
      } else if (!currentMode.isEnabled && previousMode.isEnabled) {
        compressor.disable();
      }
    }
  }

  private static void log(DTRobot robot) {
    DTWatchdog.startEpoch();
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
    robot.logRoot.log();
    DTLogger.flush();
    DTWatchdog.addEpoch("DTLog");
  }

  protected final void configCompressor(int module, PneumaticsModuleType type) {
    if (compressor != null) {
      compressor.disable();
    }
    compressor = new Compressor(module, type);
  }

  public static void runRobot(Supplier<DTRobot> robotConstructor) {
    if (!HAL.initialize(500, 0)) {
      throw new IllegalStateException("Failed to initialize HAL");
    }

    startNTServer();
    refreshDriverStation();
    DTLogger.initialize();

    DTRobot robot;
    try {
      robot = robotConstructor.get();
    } catch (Throwable t) {
      Throwable cause = t.getCause();
      if (cause != null) {
        t = cause;
      }
      System.err.println("[DTLib] Unhandled exception thrown while instantiating robot:");
      t.printStackTrace();
      return;
    }

    System.out.println("[DTLib] " + robot.getName() + " initializing...");
    robot.init();
    if (isSimulation()) {
      robot.simulationInit();
    }
    robot.bindCommands();

    DriverStationJNI.observeUserProgramStarting();
    System.out.println("[DTLib] " + robot.getName() + " ready");

    int notifierHandle = NotifierJNI.initializeNotifier();
    NotifierJNI.setNotifierName(notifierHandle, "DTRobot");
    long triggerTime = 0;
    RUN.set(true);

    while (RUN.get()) {
      // Wait to be woken up
      triggerTime += PERIOD_MICROS;
      NotifierJNI.updateNotifierAlarm(notifierHandle, triggerTime);
      long time = NotifierJNI.waitForNotifierAlarm(notifierHandle);
      if (time == 0) {
        // Notifier has been stopped, exit
        RUN.set(false);
        break;
      }

      DTWatchdog.reset();
      refreshDriverStation();
      robot.runModeChange();

      // Execute code for this cycle
      DTWatchdog.startEpoch();
      robot.periodic();
      DTWatchdog.addEpoch("periodic()");

      DTCommandScheduler.run();
      log(robot);

      if (DTWatchdog.isExpired()) {
        DriverStation.reportWarning("Loop time of " + PERIOD_SECONDS + "s overrun by "
            + (DTWatchdog.getTime() - PERIOD_SECONDS) + "s", false);
        DTWatchdog.printEpochs();
      }
    }

    HAL.shutdown();
  }

  private static void startNTServer() {
    NetworkTableInstance.getDefault()
                        .startServer();
    int count = 0;
    while (NetworkTableInstance.getDefault()
                               .getNetworkMode()
                               .contains(NetworkMode.kStarting)) {
      count++;
      if (count >= 100) {
        System.err.println("[DTLib] NT server start timeeout, continuing");
        break;
      }
      try {
        Thread.sleep(10);
      } catch (InterruptedException e) {}
    }
  }

  private static void refreshDriverStation() {
    DTWatchdog.startEpoch();
    DriverStation.refreshData();
    CONTROL_WORD.refresh();
    alliance = AllianceStation.fromDS(DriverStationJNI.getAllianceStation());

    previousMode = currentMode;
    if (CONTROL_WORD.isEStopped()) {
      currentMode = Mode.E_STOP;
      DriverStationJNI.observeUserProgramDisabled();
    } else if (!CONTROL_WORD.isEnabled()) {
      currentMode = Mode.DISABLED;
      DriverStationJNI.observeUserProgramDisabled();
    } else if (CONTROL_WORD.isAutonomous()) {
      currentMode = Mode.AUTO;
      DriverStationJNI.observeUserProgramAutonomous();
    } else if (CONTROL_WORD.isTest()) {
      currentMode = Mode.TEST;
      DriverStationJNI.observeUserProgramTest();
    } else {
      currentMode = Mode.TELEOP;
      DriverStationJNI.observeUserProgramTeleop();
    }
    DTWatchdog.addEpoch("refreshDriverStation()");
  }

  public static Mode getCurrentMode() {
    return currentMode;
  }

  public static AllianceStation getAlliance() {
    return alliance;
  }

  public static boolean isSimulation() {
    return IS_SIMULATION;
  }

  public static boolean isReal() {
    return !IS_SIMULATION;
  }

  public static boolean isDSConnected() {
    return CONTROL_WORD.isDSAttached();
  }

  public static long currentTimeMicros() {
    return HALUtil.getFPGATime();
  }

  public static double currentTime() {
    return currentTimeMicros() * 1e-6;
  }
}
