package org.victorrobotics.dtlib.log;

import org.victorrobotics.dtlib.DTLibInfo;
import org.victorrobotics.dtlib.DTRobot;

import java.io.File;
import java.io.IOException;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.nio.charset.StandardCharsets;
import java.time.Clock;
import java.time.Instant;
import java.time.LocalDate;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.WPILibVersion;

public class DTLogger {
  private static final DateTimeFormatter TIME_FORMATTER = //
      DateTimeFormatter.ofPattern("uuuu-MM-dd_HH-mm-ss")
                       .withZone(ZoneId.of("Z"));

  private static final String DTLIB_LOGGER_MAGIC = "DTLib Logger";
  private static final String LOG_DIRECTORY      = "/dev/sda/logs";

  static final Map<Class<?>, DTLogType<?>> LOGGABLE_TYPES = new HashMap<>();

  static {
    DTLogBuiltinTypes.load();
  }

  private static File        file;
  private static DTLogWriter dataWriter;

  private static long lastTimestamp;
  private static long startTimeMillis;
  private static int  nextHandle = 0x0100;

  private DTLogger() {}

  public static DTLogWriter getWriter() { return dataWriter; }

  public static int newHandle(int typeID, String path) {
    dataWriter.writeShort(typeID);
    dataWriter.writeStringUTF8(path);
    return nextHandle++;
  }

  public static boolean logNewTimestamp() {
    long newTime = DTRobot.currentTimeMicros() / 1000;
    long diff = newTime - lastTimestamp;
    if (diff <= 0) {
      // Same time, no change
      return false;
    }
    lastTimestamp = newTime;

    if (diff <= 0xFFFF) {
      // 2 byte form, maximum of 65.535 seconds
      dataWriter.writeShort(0x0001);
      dataWriter.writeShort((short) diff);
    } else {
      // write new timestamp
      dataWriter.writeShort(0x0002);
      dataWriter.writeLong(newTime);
    }
    return true;
  }

  public static void initialize() {
    while (true) {
      if (isTimeSynchronized()) {
        try {
          openWriter();
          writeHeader();
          return;
        } catch (IOException e) {}
      }

      try {
        Thread.sleep(10);
      } catch (InterruptedException e) {}
    }
  }

  private static void openWriter() throws IOException {
    Instant now = Clock.systemUTC()
                       .instant();
    lastTimestamp = DTRobot.currentTimeMicros() / 1_000;
    startTimeMillis = now.toEpochMilli();

    String name = TIME_FORMATTER.format(now) + ".dtlog";
    file = new File(LOG_DIRECTORY, name);
    dataWriter = new DTLogWriter(file, 65536);
  }

  private static void writeHeader() {
    dataWriter.writeBytes(DTLIB_LOGGER_MAGIC.getBytes(StandardCharsets.UTF_8));

    dataWriter.writeShort(DTLibInfo.Version.YEAR);
    dataWriter.writeByte(DTLibInfo.Version.MAJOR);
    dataWriter.writeByte(DTLibInfo.Version.MINOR);

    String[] wpilibVersion = WPILibVersion.Version.split("\\.");
    dataWriter.writeShort(Integer.parseInt(wpilibVersion[0]));
    dataWriter.writeByte(Integer.parseInt(wpilibVersion[1]));
    dataWriter.writeByte(Integer.parseInt(wpilibVersion[2]));

    dataWriter.writeShort(getTeamNumber());
    dataWriter.writeShort(0); // unused flags
    dataWriter.writeLong(startTimeMillis);
  }

  private static boolean isTimeSynchronized() {
    return DTRobot.isDSConnected() && LocalDate.now(Clock.systemUTC())
                                               .getYear()
        >= 2000;
  }

  public static void logDebug(String msg) {
    logMessage(msg, 0x0008);
  }

  public static void logInfo(String msg) {
    logMessage(msg, 0x0009);
  }

  public static void logWarning(String msg) {
    logMessage(msg, 0x000a);
  }

  public static void logError(String msg) {
    logMessage(msg, 0x000b);
  }

  private static void logMessage(String msg, int type) {
    dataWriter.writeShort(type);
    dataWriter.writeStringUTF8(msg);
  }

  public static void flush() {
    dataWriter.flush();
  }

  private static int getTeamNumber() {
    if (RobotBase.isSimulation()) {
      // Can't use IP address in a simulation, return unknown
      return 0xffff;
    }

    // Use IP address 10.??.??.2 to obtain team number
    try {
      Enumeration<NetworkInterface> interfaces = NetworkInterface.getNetworkInterfaces();
      while (interfaces.hasMoreElements()) {
        NetworkInterface i = interfaces.nextElement();
        Enumeration<InetAddress> addresses = i.getInetAddresses();
        while (addresses.hasMoreElements()) {
          InetAddress address = addresses.nextElement();
          int teamNum = getTeamNumber(address);
          if (teamNum != -1) {
            return teamNum;
          }
        }
      }
    } catch (SocketException e) {
      // ignore
    }

    // Team number cannot be determined, return unknown
    return 0xffff;
  }

  private static int getTeamNumber(InetAddress address) {
    String[] split = address.getHostAddress()
                            .split("\\.");
    if ("10".equals(split[0]) && "2".equals(split[3])) {
      try {
        int hundreds = Integer.parseInt(split[1]);
        int ones = Integer.parseInt(split[2]);
        if (hundreds < 100 && ones < 100) {
          return hundreds * 100 + ones;
        }
      } catch (NumberFormatException e) {
        // ignore
      }
    }
    return -1;
  }
}
