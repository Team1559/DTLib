package org.victorrobotics.dtlib.log;

import org.victorrobotics.dtlib.DTRobot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.DriverStation;

public final class DTWatchdog {
  private static class Epoch {
    private final String label;
    private final long   duration;

    Epoch(String label, long durationMicros) {
      this.label = label;
      this.duration = durationMicros;
    }
  }

  private static final long MIN_PRINT_DELAY = 1_000_000;
  private static final long DEFAULT_TIMEOUT = (long) (0.02 * 1e6);

  private static final List<Epoch> EPOCHS = new ArrayList<>();

  private static long period = DEFAULT_TIMEOUT;

  private static long epochStartTime;
  private static long loopStartTime;
  private static long loopExpireTime;
  private static long minPrintTime;

  private DTWatchdog() {}

  public static double getTime() { return (DTRobot.currentTimeMicros() - loopStartTime) * 1e-6; }

  public static double getPeriod() { return period * 1e-6; }

  public static void reset() {
    loopStartTime = DTRobot.currentTimeMicros();
    loopExpireTime = loopStartTime + period;
    epochStartTime = loopStartTime;
    EPOCHS.clear();
  }

  public static void addEpoch(String label) {
    long time = DTRobot.currentTimeMicros();
    EPOCHS.add(new Epoch(label, time - epochStartTime));
    epochStartTime = time;
  }

  public static void printEpochs() {
    printEpochs(str -> DriverStation.reportWarning(str, false));
  }

  public static void printEpochs(Consumer<String> action) {
    long time = DTRobot.currentTimeMicros();
    if (time < minPrintTime) return;
    minPrintTime = time + MIN_PRINT_DELAY;

    StringBuilder builder =
        new StringBuilder("Loop Overrun: ").append((time - loopStartTime) * 1e-6)
                                           .append(" seconds")
                                           .append(System.lineSeparator());

    int labelLength = 0;
    for (Epoch epoch : EPOCHS) {
      int len = epoch.label.length();
      if (len > labelLength) {
        labelLength = len;
      }
    }
    String format = "%-" + labelLength + "s - %.6f%n";
    EPOCHS.forEach(epoch -> builder.append(String.format(format, epoch.label,
                                                         epoch.duration * 1e-6)));

    action.accept(builder.toString());
  }

  public static void startEpoch() {
    epochStartTime = DTRobot.currentTimeMicros();
  }

  public static boolean isExpired() { return DTRobot.currentTimeMicros() >= loopExpireTime; }

  public static void setPeriod(double periodSeconds) { period = (long) (periodSeconds * 1e6); }
}
