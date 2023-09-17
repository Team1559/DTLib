package org.victorrobotics.dtlib.log;

import org.victorrobotics.dtlib.DTLibInfo;
import org.victorrobotics.dtlib.exception.DTIllegalArgumentException;

import static java.nio.file.StandardOpenOption.CREATE;
import static java.nio.file.StandardOpenOption.TRUNCATE_EXISTING;
import static java.nio.file.StandardOpenOption.WRITE;

import java.io.Closeable;
import java.io.File;
import java.io.Flushable;
import java.io.IOException;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.time.Clock;
import java.time.Instant;
import java.time.LocalDate;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.util.BitSet;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.WPILibVersion;

public final class DTLogWriter implements Closeable, Flushable {
  private static final DateTimeFormatter TIME_FORMATTER = // UTC
      DateTimeFormatter.ofPattern("uuuu-MM-dd_HH-mm-ss")
                       .withZone(ZoneId.of("Z"));

  private static final String LOG_DIRECTORY      = "/dev/sda/logs";
  private static final byte[] HEADER_MAGIC_BYTES = "DTLib Logger".getBytes(StandardCharsets.UTF_8);

  // @format:off
  private static final int HEADER_MAGIC_XOR =
      (('D' ^ 'b' ^ 'g') << 24) |
      (('T' ^ ' ' ^ 'g') << 16) |
      (('L' ^ 'L' ^ 'e') <<  8) |
      (('i' ^ 'o' ^ 'r') <<  0);
  // @format:on

  private static final int BUFFER_SIZE         = 65536;
  private static final int UNKNOWN_TEAM_NUMBER = 0xFFFF;

  private static DTLogWriter INSTANCE;

  static final Map<Class<?>, DTLogType> LOG_TYPES = new HashMap<>();

  static {
    DTLogBuiltinTypes.load();
  }

  private final File        file;
  private final FileChannel channel;
  private final ByteBuffer  buffer;
  private final BitSet      bitSet;

  private long lastTimestamp;
  private int  nextVarHandle = 0x0100;

  private DTLogWriter() throws IOException {
    Instant now = Clock.systemUTC()
                       .instant();
    lastTimestamp = RobotController.getFPGATime() / 1_000;
    long startTimeMillis = now.toEpochMilli();
    String name = "LOG_" + TIME_FORMATTER.format(now) + ".dtlog";

    file = new File(LOG_DIRECTORY, name);
    channel = FileChannel.open(file.toPath(), WRITE, CREATE, TRUNCATE_EXISTING);
    buffer = ByteBuffer.allocateDirect(BUFFER_SIZE);
    bitSet = new BitSet();

    writeBytes(HEADER_MAGIC_BYTES);
    int checksum = HEADER_MAGIC_XOR;

    int dtlibVersion = (DTLibInfo.Version.YEAR << 16) | (DTLibInfo.Version.MAJOR << 8)
        | DTLibInfo.Version.MINOR;
    writeInt(dtlibVersion);
    checksum ^= dtlibVersion;

    String[] wpilibVersions = WPILibVersion.Version.split("\\.");
    int wpilibYear = Integer.parseInt(wpilibVersions[0]);
    int wpilibMajor = Integer.parseInt(wpilibVersions[1]);
    int wpilibMinor = Integer.parseInt(wpilibVersions[2]);
    int wpilibVersion = (wpilibYear << 16) | (wpilibMajor << 8) | wpilibMinor;
    writeInt(wpilibVersion);
    checksum ^= wpilibVersion;

    int team = getTeamNumber();
    writeLong(((long) team << 48) | startTimeMillis);
    checksum ^= team << 16;
    checksum ^= (int) ((startTimeMillis >> 32) | startTimeMillis);

    writeInt(checksum);
    flush();
  }

  public DTLogWriter writeByte(int b) {
    checkBufferRemaining(1);
    buffer.put((byte) b);
    return this;
  }

  public DTLogWriter writeShort(int s) {
    checkBufferRemaining(2);
    buffer.putShort((short) s);
    return this;
  }

  public DTLogWriter writeInt(int i) {
    checkBufferRemaining(4);
    buffer.putInt(i);
    return this;
  }

  public DTLogWriter writeLong(long l) {
    checkBufferRemaining(8);
    buffer.putLong(l);
    return this;
  }

  public DTLogWriter writeDouble(double d) {
    checkBufferRemaining(8);
    buffer.putDouble(d);
    return this;
  }

  public DTLogWriter writeFloat(float f) {
    checkBufferRemaining(4);
    buffer.putFloat(f);
    return this;
  }

  public DTLogWriter writeChar(char c) {
    checkBufferRemaining(2);
    buffer.putChar(c);
    return this;
  }

  public DTLogWriter writeBoolean(boolean b) {
    checkBufferRemaining(1);
    buffer.put((byte) (b ? 1 : 0));
    return this;
  }

  public DTLogWriter writeBytes(byte[] b) {
    checkBufferRemaining(b.length);
    buffer.put(b);
    return this;
  }

  public DTLogWriter writeByteArray(byte[] b) {
    checkWriteArrayLength(b.length);
    checkBufferRemaining(b.length);
    buffer.put(b);
    return this;
  }

  public DTLogWriter writeShorts(short[] s) {
    checkBufferRemaining(s.length * 2);
    for (int i = 0; i < s.length; i++) {
      buffer.putShort(s[i]);
    }
    return this;
  }

  public DTLogWriter writeShortArray(short[] s) {
    checkWriteArrayLength(s.length);
    checkBufferRemaining(s.length * 2);
    for (int i = 0; i < s.length; i++) {
      buffer.putShort(s[i]);
    }
    return this;
  }

  public DTLogWriter writeInts(int[] i) {
    checkBufferRemaining(i.length * 4);
    for (int j = 0; j < i.length; j++) {
      buffer.putInt(i[j]);
    }
    return this;
  }

  public DTLogWriter writeIntArray(int[] i) {
    checkWriteArrayLength(i.length);
    checkBufferRemaining(i.length * 4);
    for (int j = 0; j < i.length; j++) {
      buffer.putInt(i[j]);
    }
    return this;
  }

  public DTLogWriter writeLongs(long[] l) {
    checkBufferRemaining(l.length * 8);
    for (int i = 0; i < l.length; i++) {
      buffer.putLong(l[i]);
    }
    return this;
  }

  public DTLogWriter writeLongArray(long[] l) {
    checkWriteArrayLength(l.length);
    checkBufferRemaining(l.length * 8);
    for (int i = 0; i < l.length; i++) {
      buffer.putLong(l[i]);
    }
    return this;
  }

  public DTLogWriter writeDoubles(double[] d) {
    checkBufferRemaining(d.length * 8);
    for (int i = 0; i < d.length; i++) {
      buffer.putDouble(d[i]);
    }
    return this;
  }

  public DTLogWriter writeDoubleArray(double[] d) {
    checkWriteArrayLength(d.length);
    checkBufferRemaining(d.length * 8);
    for (int i = 0; i < d.length; i++) {
      buffer.putDouble(d[i]);
    }
    return this;
  }

  public DTLogWriter writeFloats(float[] f) {
    checkBufferRemaining(f.length * 4);
    for (int i = 0; i < f.length; i++) {
      buffer.putDouble(f[i]);
    }
    return this;
  }

  public DTLogWriter writeFloatArray(float[] f) {
    checkWriteArrayLength(f.length);
    checkBufferRemaining(f.length * 4);
    for (int i = 0; i < f.length; i++) {
      buffer.putDouble(f[i]);
    }
    return this;
  }

  public DTLogWriter writeChars(char[] c) {
    checkBufferRemaining(c.length * 2);
    for (int i = 0; i < c.length; i++) {
      buffer.putChar(c[i]);
    }
    return this;
  }

  public DTLogWriter writeCharArray(char[] c) {
    checkWriteArrayLength(c.length);
    checkBufferRemaining(c.length * 2);
    for (int i = 0; i < c.length; i++) {
      buffer.putChar(c[i]);
    }
    return this;
  }

  public DTLogWriter writeBooleans(boolean[] b) {
    int len = (b.length + 7) / 8;
    checkBufferRemaining(len);
    byte[] data = booleansToBytes(b);
    buffer.put(data, 0, len);
    return this;
  }

  public DTLogWriter writeBooleanArray(boolean[] b) {
    int len = (b.length + 7) / 8;
    checkWriteArrayLength(b.length);
    checkBufferRemaining(len);
    byte[] bytes = booleansToBytes(b);
    buffer.put(bytes, 0, len);
    return this;
  }

  private byte[] booleansToBytes(boolean[] b) {
    bitSet.set(b.length - 1); // Ensure capacity
    bitSet.clear();

    for (int i = 0; i < b.length; i++) {
      if (b[i]) {
        bitSet.set(i);
      }
    }
    return bitSet.toByteArray();
  }

  public DTLogWriter writeStringUTF8(String s) {
    byte[] data = s.getBytes(StandardCharsets.UTF_8);
    writeByteArray(data);
    return this;
  }

  @Override
  public void close() throws IOException {
    channel.close();
  }

  @Override
  public void flush() throws IOException {
    buffer.flip();
    channel.write(buffer);
    buffer.compact();
  }

  public boolean tryFlush() {
    try {
      flush();
      return true;
    } catch (IOException e) {
      // TODO: don't ignore this
      return false;
    }
  }

  private boolean checkBufferRemaining(int newDataLength) {
    if (buffer.capacity() < newDataLength) {
      tryFlush();
      return false;
    }

    while (buffer.remaining() < newDataLength) {
      tryFlush();
    }
    return true;
  }

  private void checkWriteArrayLength(int len) {
    if (len > 0xFFFF) {
      throw new DTIllegalArgumentException(len, "array is too large to log");
    }
    writeShort(len);
  }

  int declareNewVariableHandle(int typeID, String path) {
    writeShort(typeID);
    writeStringUTF8(path);
    return nextVarHandle++;
  }

  public boolean logNewTimestamp() {
    long newTime = RobotController.getFPGATime() / 1000;
    long diff = newTime - lastTimestamp;
    if (diff <= 0) {
      // Same time, no change
      return false;
    }
    lastTimestamp = newTime;

    if (diff <= 0xFFFF) {
      // write increment, maximum of 65.535 seconds
      INSTANCE.writeInt((0x01 << 16) | (int) diff);
    } else {
      // write new timestamp
      INSTANCE.writeLong((0x02L << 48) | newTime);
    }
    return true;
  }

  public void logMessage(String msg, DTLog.Level level) {
    writeShort(level.typeID);
    writeStringUTF8(msg);
  }

  public static void init() {
    while (true) {
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        // ignore
      }

      if (!isTimeSynchronized()) {
        continue;
      }

      try {
        INSTANCE = new DTLogWriter();
        return;
      } catch (IOException e) {
        // ignore
      }
    }
  }

  private static boolean isTimeSynchronized() {
    return DriverStation.isDSAttached() && LocalDate.now(Clock.systemUTC())
                                                    .getYear() >= 2000;
  }

  public static DTLogWriter getInstance() {
    return INSTANCE;
  }

  private static int getTeamNumber() {
    if (RobotBase.isSimulation()) {
      return UNKNOWN_TEAM_NUMBER;
    }

    Enumeration<NetworkInterface> interfaces;
    try {
      interfaces = NetworkInterface.getNetworkInterfaces();
    } catch (SocketException e) {
      return UNKNOWN_TEAM_NUMBER;
    }

    // Use IP address 10.??.??.2 to obtain team number
    while (interfaces.hasMoreElements()) {
      NetworkInterface i = interfaces.nextElement();
      Enumeration<InetAddress> addresses = i.getInetAddresses();
      while (addresses.hasMoreElements()) {
        String[] split = addresses.nextElement()
                                  .getHostAddress()
                                  .split("\\.");
        if (!"10".equals(split[0]) || !"41".equals(split[3])) {
          continue;
        }

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
    }

    return UNKNOWN_TEAM_NUMBER;
  }
}
