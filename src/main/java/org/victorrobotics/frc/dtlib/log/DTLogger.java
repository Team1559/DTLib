package org.victorrobotics.frc.dtlib.log;

import org.victorrobotics.frc.dtlib.DTLibInfo;

import java.io.IOException;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.channels.SeekableByteChannel;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.time.Clock;
import java.time.Instant;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;
import java.util.regex.Pattern;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.WPILibVersion;

public class DTLogger {
    private static final ZoneId            TIME_ZONE      = ZoneId.of("UTC");
    private static final DateTimeFormatter TIME_FORMATTER = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss")
                                                                             .withZone(TIME_ZONE);

    private static final Pattern SPLIT_BY_PERIODS = Pattern.compile("\\.");

    private static final byte[] DTLIB_LOGGER_BYTES = { 0x44, 0x54, 0x4c, 0x69, 0x62, 0x20, 0x4c, 0x6f, 0x67, 0x67, 0x65,
            0x72 };

    private static String LOG_DIRECTORY = "/dev/sda1/dtlog";

    private static final List<DTLogVar> entries = new ArrayList<>();

    static final DTLogWriter       dataWriter;
    private static SeekableByteChannel fileChannel;

    private static Path filepath;

    private static String filename;
    private static long   lastTimestamp;
    private static long   startTimeMicros;
    private static int    dsAttachCount;
    private static int    nextHandle;

    static {
        // 64 KB
        dataWriter = new DTLogWriter(ByteBuffer.allocateDirect(65536));
    }

    private DTLogger() {}

    public static int generateHandle() {
        return nextHandle++;
    }

    public static void logNewTimestamp() {
        long newTime = getFPGATimeMS();
        long diff = newTime - lastTimestamp;
        if (diff <= 0) {
            // Same time, no change
            return;
        }
        lastTimestamp = newTime;

        if (diff <= 0xFF) {
            // 1 byte form, maximum of 0.255 seconds
            dataWriter.writeByte(0x02);
            dataWriter.writeByte((int) diff);
        } else {
            // 4 byte form, maximum of 49.7 days
            dataWriter.writeByte(0x03);
            dataWriter.writeInt((int) diff);
        }
    }

    public static boolean isReady() {
        if (fileChannel != null) {
            return true;
        } else if (!isTimeSynchronized()) {
            return false;
        }

        try {
            openCapture();
            writeHeader();
        } catch (IOException e) {
            try {
                if (fileChannel != null) {
                    fileChannel.close();
                }
            } catch (IOException e2) {
                // If we can't write, and we can't close... throw everything
                e2.addSuppressed(e);
                throw new IllegalStateException(e2);
            }
            fileChannel = null;
            return false;
        }
        return true;
    }

    private static void openCapture() throws IOException {
        Instant now = Clock.system(TIME_ZONE)
                           .instant();
        lastTimestamp = RobotController.getFPGATime() / 1_000;
        startTimeMicros = now.getEpochSecond() * 1_000_000 + now.getNano() / 1_000;

        filename = "DTLog_" + TIME_FORMATTER.format(LocalDateTime.ofInstant(now, TIME_ZONE)) + ".dtlog";
        filepath = Paths.get(LOG_DIRECTORY, filename);

        fileChannel = Files.newByteChannel(filepath, StandardOpenOption.WRITE, StandardOpenOption.CREATE,
                StandardOpenOption.TRUNCATE_EXISTING);
    }

    private static void writeHeader() {
        dataWriter.writeBytes(DTLIB_LOGGER_BYTES);

        dataWriter.writeShort(DTLibInfo.Version.YEAR);
        dataWriter.writeByte(DTLibInfo.Version.MAJOR);
        dataWriter.writeByte(DTLibInfo.Version.MINOR);

        String[] wpilibVersion = SPLIT_BY_PERIODS.split(WPILibVersion.Version);
        dataWriter.writeShort(Short.parseShort(wpilibVersion[0]));
        dataWriter.writeByte(Integer.parseInt(wpilibVersion[1]));
        dataWriter.writeByte(Integer.parseInt(wpilibVersion[2]));

        dataWriter.writeShort(getTeamNumber());

        dataWriter.writeShort(0); // unused flags

        dataWriter.writeLong(startTimeMicros);
    }

    private static boolean isTimeSynchronized() {
        if (DriverStation.isDSAttached()) {
            dsAttachCount++;
            // Connected for 0.2s & correct year
            return dsAttachCount >= 10 && LocalDateTime.now(TIME_ZONE)
                                                       .getYear() >= 2000;
        } else {
            dsAttachCount = 0;
            return false;
        }
    }

    public static void logInfo(String msg) {
        logMessage(msg, 0x0c);
    }

    public static void logWarning(String msg) {
        logMessage(msg, 0x0d);
    }

    public static void logError(String msg) {
        logMessage(msg, 0x0e);
    }

    public static void logFatal(String msg) {
        logMessage(msg, 0x0f);
    }

    // Returns whether it succeeded. If it failed, nothing was written
    private static void logMessage(String msg, int type) {
        dataWriter.writeByte(type);
        dataWriter.writeStringUTF(msg);
    }

    private static long getFPGATimeMS() {
        long fpga = RobotController.getFPGATime();
        long time = fpga / 1000;
        // Round to nearest ms
        return (fpga - (time * 1000) >= 500) ? (time + 1) : time;
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
        String[] split = SPLIT_BY_PERIODS.split(address.getHostAddress());
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
