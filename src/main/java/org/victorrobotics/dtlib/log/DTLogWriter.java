package org.victorrobotics.dtlib.log;

import org.victorrobotics.dtlib.exception.DTIllegalArgumentException;

import java.io.Closeable;
import java.io.File;
import java.io.Flushable;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.StandardOpenOption;
import java.util.BitSet;

public class DTLogWriter implements Closeable, Flushable {
  private final FileChannel channel;
  private final ByteBuffer  buffer;
  private final BitSet      bitSet;

  public DTLogWriter(File file, int bufferSize) throws IOException {
    this.channel = FileChannel.open(file.toPath(), StandardOpenOption.WRITE,
        StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING);
    this.buffer = ByteBuffer.allocateDirect(bufferSize);
    this.bitSet = new BitSet();
  }

  public DTLogWriter writeByte(int b) {
    writeByte((byte) b);
    return this;
  }

  public DTLogWriter writeByte(byte b) {
    checkBuffer(1);
    buffer.put(b);
    return this;
  }

  public DTLogWriter writeShort(int s) {
    writeShort((short) s);
    return this;
  }

  public DTLogWriter writeShort(short s) {
    checkBuffer(2);
    buffer.putShort(s);
    return this;
  }

  public DTLogWriter writeInt(int i) {
    checkBuffer(4);
    buffer.putInt(i);
    return this;
  }

  public DTLogWriter writeLong(long l) {
    checkBuffer(8);
    buffer.putLong(l);
    return this;
  }

  public DTLogWriter writeDouble(float d) {
    writeDouble((double) d);
    return this;
  }

  public DTLogWriter writeDouble(double d) {
    checkBuffer(8);
    buffer.putDouble(d);
    return this;
  }

  public DTLogWriter writeFloat(double f) {
    writeFloat((float) f);
    return this;
  }

  public DTLogWriter writeFloat(float f) {
    checkBuffer(4);
    buffer.putFloat(f);
    return this;
  }

  public DTLogWriter writeChar(char c) {
    checkBuffer(2);
    buffer.putChar(c);
    return this;
  }

  public DTLogWriter writeBoolean(boolean b) {
    checkBuffer(1);
    buffer.put((byte) (b ? 1 : 0));
    return this;
  }

  public DTLogWriter writeBytes(byte[] b) {
    checkBuffer(b.length);
    buffer.put(b);
    return this;
  }

  public DTLogWriter writeByteArray(byte[] b) {
    checkBuffer(b.length + 2);
    checkWriteArrayLength(b.length);
    buffer.put(b);
    return this;
  }

  public DTLogWriter writeShorts(short[] s) {
    checkBuffer(s.length * 2);
    for (int i = 0; i < s.length; i++) {
      buffer.putShort(s[i]);
    }
    return this;
  }

  public DTLogWriter writeShortArray(short[] s) {
    checkBuffer(s.length * 2 + 2);
    checkWriteArrayLength(s.length);
    for (int i = 0; i < s.length; i++) {
      buffer.putShort(s[i]);
    }
    return this;
  }

  public DTLogWriter writeInts(int[] i) {
    checkBuffer(i.length * 4);
    for (int j = 0; j < i.length; j++) {
      buffer.putInt(i[j]);
    }
    return this;
  }

  public DTLogWriter writeIntArray(int[] i) {
    checkBuffer(i.length * 4 + 2);
    checkWriteArrayLength(i.length);
    for (int j = 0; j < i.length; j++) {
      buffer.putInt(i[j]);
    }
    return this;
  }

  public DTLogWriter writeLongs(long[] l) {
    checkBuffer(l.length * 8);
    for (int i = 0; i < l.length; i++) {
      buffer.putLong(l[i]);
    }
    return this;
  }

  public DTLogWriter writeLongArray(long[] l) {
    checkBuffer(l.length * 8 + 2);
    checkWriteArrayLength(l.length);
    for (int i = 0; i < l.length; i++) {
      buffer.putLong(l[i]);
    }
    return this;
  }

  public DTLogWriter writeDoubles(double[] d) {
    checkBuffer(d.length * 8);
    for (int i = 0; i < d.length; i++) {
      buffer.putDouble(d[i]);
    }
    return this;
  }

  public DTLogWriter writeDoubleArray(double[] d) {
    checkBuffer(d.length * 8 + 2);
    checkWriteArrayLength(d.length);
    for (int i = 0; i < d.length; i++) {
      buffer.putDouble(d[i]);
    }
    return this;
  }

  public DTLogWriter writeFloats(float[] f) {
    checkBuffer(f.length * 4);
    for (int i = 0; i < f.length; i++) {
      buffer.putDouble(f[i]);
    }
    return this;
  }

  public DTLogWriter writeFloatArray(float[] f) {
    checkBuffer(f.length * 4 + 2);
    checkWriteArrayLength(f.length);
    for (int i = 0; i < f.length; i++) {
      buffer.putDouble(f[i]);
    }
    return this;
  }

  public DTLogWriter writeChars(char[] c) {
    checkBuffer(c.length * 2);
    for (int i = 0; i < c.length; i++) {
      buffer.putChar(c[i]);
    }
    return this;
  }

  public DTLogWriter writeCharArray(char[] c) {
    checkBuffer(c.length * 2 + 2);
    checkWriteArrayLength(c.length);
    for (int i = 0; i < c.length; i++) {
      buffer.putChar(c[i]);
    }
    return this;
  }

  public DTLogWriter writeBooleans(boolean[] b) {
    int len = (b.length + 7) / 8;
    checkBuffer(len);
    byte[] data = booleansToBytes(b);
    buffer.put(data, 0, len);
    return this;
  }

  public DTLogWriter writeBooleanArray(boolean[] b) {
    int len = (b.length + 7) / 8;
    checkBuffer(len + 2);
    checkWriteArrayLength(b.length);
    byte[] bytes = booleansToBytes(b);
    buffer.put(bytes, 0, len);
    return this;
  }

  private byte[] booleansToBytes(boolean[] b) {
    bitSet.clear();
    for (int i = b.length - 1; i >= 0; i--) {
      bitSet.set(i, b[i]);
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
  public void flush() {
    try {
      int bufferPos = buffer.position();
      long channelPos = -1;
      buffer.flip();
      try {
        channelPos = channel.position();
        channel.write(buffer);
      } catch (IOException e) {
        // Error while writing, attempt to recover
        buffer.limit(buffer.capacity());
        buffer.position(bufferPos);
        channel.position(channelPos);
      }
      buffer.compact();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  private boolean checkBuffer(int newDataLength) {
    int remaining = buffer.remaining();
    if (remaining >= newDataLength) {
      // Already enough space
      return false;
    }

    do {
      flush();
      remaining = buffer.remaining();
    } while (remaining < newDataLength);
    return true;
  }

  private void checkWriteArrayLength(int len) {
    if (len > 0xFFFF) {
      throw new DTIllegalArgumentException(len, "array is too large to log");
    }
    buffer.putShort((short) len);
  }
}
