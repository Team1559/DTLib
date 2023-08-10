package org.victorrobotics.frc.dtlib.log;

import org.victorrobotics.frc.dtlib.exception.DTIllegalArgumentException;

import java.io.Closeable;
import java.io.Flushable;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.ClosedChannelException;
import java.nio.channels.SeekableByteChannel;
import java.nio.charset.StandardCharsets;
import java.util.BitSet;

public class DTDataLogWriter implements Closeable, Flushable {
    private final ByteBuffer buffer;
    private final BitSet     bitSet;

    private SeekableByteChannel fileChannel;

    public DTDataLogWriter(ByteBuffer buffer) {
        this.buffer = buffer;
        this.bitSet = new BitSet();
    }

    @Override
    public void close() throws IOException {
        if (fileChannel != null) {
            fileChannel.close();
        }
    }

    public void setFileChannel(SeekableByteChannel channel) {
        fileChannel = channel;
    }

    public SeekableByteChannel getFileChannel() {
        return fileChannel;
    }

    public void flush() {
        int dataLength = buffer.position();
        long fileStartPosition = -1;
        try {
            fileStartPosition = fileChannel.position();
            buffer.flip();
            fileChannel.write(buffer);
            buffer.compact(); // in case not everything was written
        } catch (ClosedChannelException e) {
            throw new IllegalStateException(e);
        } catch (IOException e) {
            // Error while writing, attempt to recover
            buffer.limit(buffer.capacity());
            buffer.position(dataLength);
            try {
                fileChannel.truncate(fileStartPosition);
            } catch (IOException e2) {
                // Can't recover now, state is indeterminate
                throw new IllegalStateException(e2.initCause(e));
            }
        }
    }

    public ByteBuffer getBuffer() {
        return buffer;
    }

    public DTDataLogWriter writeByte(int b) {
        flushIfNecessary(1);

        buffer.put((byte) b);
        return this;
    }

    public DTDataLogWriter writeByte(byte b) {
        flushIfNecessary(1);

        buffer.put(b);
        return this;
    }

    public DTDataLogWriter writeBytes(byte[] b) {
        flushIfNecessary(b.length);

        buffer.put(b);
        return this;
    }

    public DTDataLogWriter writeByteArray(byte[] b) {
        checkArrayLength(b.length);
        flushIfNecessary(b.length + 1);

        buffer.put((byte) b.length)
              .put(b);
        return this;
    }

    public DTDataLogWriter writeShort(int s) {
        flushIfNecessary(2);

        buffer.putShort((short) s);
        return this;
    }

    public DTDataLogWriter writeShort(short s) {
        flushIfNecessary(2);

        buffer.putShort(s);
        return this;
    }

    public DTDataLogWriter writeShorts(short[] s) {
        flushIfNecessary(s.length * 2);

        buffer.asShortBuffer()
              .put(s);
        return this;
    }

    public DTDataLogWriter writeShortArray(short[] s) {
        checkArrayLength(s.length);
        flushIfNecessary(s.length * 2 + 1);

        buffer.put((byte) s.length)
              .asShortBuffer()
              .put(s);
        return this;
    }

    public DTDataLogWriter writeInt(int i) {
        flushIfNecessary(4);

        buffer.putInt(i);
        return this;
    }

    public DTDataLogWriter writeInts(int[] i) {
        flushIfNecessary(i.length * 4);

        buffer.asIntBuffer()
              .put(i);
        return this;
    }

    public DTDataLogWriter writeIntArray(int[] i) {
        checkArrayLength(i.length);
        flushIfNecessary(i.length * 4 + 1);

        buffer.put((byte) i.length)
              .asIntBuffer()
              .put(i);
        return this;
    }

    public DTDataLogWriter writeLong(long l) {
        flushIfNecessary(8);

        buffer.putLong(l);
        return this;
    }

    public DTDataLogWriter writeLongs(long[] l) {
        flushIfNecessary(l.length * 8);
        buffer.asLongBuffer()
              .put(l);
        return this;
    }

    public DTDataLogWriter writeLongArray(long[] l) {
        checkArrayLength(l.length);
        flushIfNecessary(l.length * 8 + 1);

        buffer.put((byte) l.length)
              .asLongBuffer()
              .put(l);
        return this;
    }

    public DTDataLogWriter writeDouble(float d) {
        flushIfNecessary(8);

        buffer.putDouble(d);
        return this;
    }

    public DTDataLogWriter writeDouble(double d) {
        flushIfNecessary(8);

        buffer.putDouble(d);
        return this;
    }

    public DTDataLogWriter writeDoubles(double[] d) {
        flushIfNecessary(d.length * 8);

        buffer.asDoubleBuffer()
              .put(d);
        return this;
    }

    public DTDataLogWriter writeDoubleArray(double[] d) {
        checkArrayLength(d.length);
        flushIfNecessary(d.length * 8 + 1);

        buffer.put((byte) d.length)
              .asDoubleBuffer()
              .put(d);
        return this;
    }

    public DTDataLogWriter writeFloat(float f) {
        flushIfNecessary(4);
        buffer.putFloat(f);
        return this;
    }

    public DTDataLogWriter writeFloat(double f) {
        flushIfNecessary(4);

        buffer.putFloat((float) f);
        return this;
    }

    public DTDataLogWriter writeFloats(float[] f) {
        flushIfNecessary(f.length * 4);

        buffer.asFloatBuffer()
              .put(f);
        return this;
    }

    public DTDataLogWriter writeFloatArray(float[] f) {
        checkArrayLength(f.length);
        flushIfNecessary(f.length * 4 + 1);

        buffer.put((byte) f.length)
              .asFloatBuffer()
              .put(f);
        return this;
    }

    public DTDataLogWriter writeChar(char c) {
        flushIfNecessary(2);

        buffer.putChar(c);
        return this;
    }

    public DTDataLogWriter writeChars(char[] c) {
        flushIfNecessary(c.length * 2);

        buffer.asCharBuffer()
              .put(c);
        return this;
    }

    public DTDataLogWriter writeCharArray(char[] c) {
        checkArrayLength(c.length);
        flushIfNecessary(c.length * 2 + 1);

        buffer.put((byte) c.length)
              .asCharBuffer()
              .put(c);
        return this;
    }

    public DTDataLogWriter writeBoolean(boolean b) {
        flushIfNecessary(1);

        buffer.put((byte) (b ? 1 : 0));
        return this;
    }

    private byte[] booleansToBytes(boolean[] b) {
        bitSet.set(b.length); // Ensure capacity
        for (int i = 0; i < b.length; i++) {
            bitSet.set(i, b[i]);
        }
        return bitSet.toByteArray();
    }

    public DTDataLogWriter writeBooleans(boolean[] b) {
        int len = (b.length + 7) / 8;
        flushIfNecessary(len);

        byte[] data = booleansToBytes(b);
        buffer.put(data, 0, len);
        return this;
    }

    public DTDataLogWriter writeBooleanArray(boolean[] b) {
        checkArrayLength(b.length);
        int len = (b.length + 7) / 8;
        flushIfNecessary(len + 1);

        byte[] bytes = booleansToBytes(b);
        buffer.put((byte) b.length)
              .put(bytes, 0, len);
        return this;
    }

    public DTDataLogWriter writeStringUTF(String s) {
        byte[] data = s.getBytes(StandardCharsets.UTF_8);
        return writeByteArray(data);
    }

    private boolean flushIfNecessary(int newDataLength) {
        int remaining = buffer.remaining();
        if (remaining >= newDataLength) {
            // Already enough space
            return false;
        }

        // In case there's not enough space the first time
        for (int i = 0; i < 5; i++) {
            flush();
            int newRemaining = buffer.remaining();
            if (newRemaining > remaining) {
                if (newRemaining >= newDataLength) {
                    // Enough space now
                    return true;
                }
                i = 0;
                remaining = newRemaining;
            }
        }

        // 5 consecutive write failures
        throw new IllegalStateException("Failed to create enough space for new data");
    }

    private static void checkArrayLength(int len) {
        if (len > 0xFF) {
            throw new DTIllegalArgumentException("array is too large to log", len);
        }
    }
}
