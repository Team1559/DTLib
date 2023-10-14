package org.victorrobotics.dtlib.log;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;

public final class BuiltinLogTypes {
  private BuiltinLogTypes() {}

  static void load() {
    loadPrimitives();
    loadJava();
    loadWPILib();
    loadDTLib();
  }

  private static void loadPrimitives() {
    new LogType((Boolean b) -> {
      LogWriter.getInstance()
                 .writeBoolean(b.booleanValue());
    }, 0x20, Set.of(Boolean.class, boolean.class));

    new LogType((Byte b) -> {
      LogWriter.getInstance()
                 .writeByte(b.byteValue());
    }, 0x21, Set.of(Byte.class, byte.class));

    new LogType((Character c) -> {
      LogWriter.getInstance()
                 .writeChar(c.charValue());
    }, 0x22, Set.of(Character.class, char.class));

    new LogType((Double d) -> {
      LogWriter.getInstance()
                 .writeDouble(d.doubleValue());
    }, 0x23, Set.of(Double.class, double.class));

    new LogType((Float f) -> {
      LogWriter.getInstance()
                 .writeFloat(f.floatValue());
    }, 0x24, Set.of(Float.class, float.class));

    new LogType((Integer i) -> {
      LogWriter.getInstance()
                 .writeInt(i.intValue());
    }, 0x25, Set.of(Integer.class, int.class));

    new LogType((Long l) -> {
      LogWriter.getInstance()
                 .writeLong(l.longValue());
    }, 0x26, Set.of(Long.class, long.class));

    new LogType((Short s) -> {
      LogWriter.getInstance()
                 .writeShort(s.shortValue());
    }, 0x27, Set.of(Short.class, short.class));

    // 1D primitive arrays (special case)
    new LogType((boolean[] b) -> {
      LogWriter.getInstance()
                 .writeBooleanArray(b);
    }, 0x28, boolean[].class);

    new LogType((byte[] b) -> {
      LogWriter.getInstance()
                 .writeByteArray(b);
    }, 0x29, byte[].class);

    new LogType((char[] b) -> {
      LogWriter.getInstance()
                 .writeCharArray(b);
    }, 0x2A, char[].class);

    new LogType((double[] b) -> {
      LogWriter.getInstance()
                 .writeDoubleArray(b);
    }, 0x2B, double[].class);

    new LogType((float[] b) -> {
      LogWriter.getInstance()
                 .writeFloatArray(b);
    }, 0x2C, float[].class);

    new LogType((int[] b) -> {
      LogWriter.getInstance()
                 .writeIntArray(b);
    }, 0x2D, int[].class);

    new LogType((long[] b) -> {
      LogWriter.getInstance()
                 .writeLongArray(b);
    }, 0x2E, long[].class);

    new LogType((short[] b) -> {
      LogWriter.getInstance()
                 .writeShortArray(b);
    }, 0x2F, short[].class);
  }

  private static void loadJava() {
    new LogType((String s) -> {
      LogWriter.getInstance()
                 .writeStringUTF8(s);
    }, 0x30, String.class);
  }

  private static void loadWPILib() {
    new LogType((Rotation2d r) -> {
      LogWriter.getInstance()
                 .writeDouble(r.getRadians());
    }, 0x000000, Rotation2d.class);

    new LogType((Rotation3d r) -> {
      Quaternion q = r.getQuaternion();
      LogWriter.getInstance()
                 .writeDouble(q.getW())
                 .writeDouble(q.getX())
                 .writeDouble(q.getY())
                 .writeDouble(q.getZ());
    }, 0x000000, Rotation3d.class);

    new LogType((Translation2d t) -> {
      LogWriter.getInstance()
                 .writeDouble(t.getX())
                 .writeDouble(t.getY());
    }, 0x000000, Translation2d.class);

    new LogType((Translation3d t) -> {
      LogWriter.getInstance()
                 .writeDouble(t.getX())
                 .writeDouble(t.getY())
                 .writeDouble(t.getZ());
    }, 0x000000, Translation3d.class);

    new LogType((Pose2d p) -> {
      Translation2d t = p.getTranslation();
      LogWriter.getInstance()
                 .writeDouble(t.getX())
                 .writeDouble(t.getY())
                 .writeDouble(p.getRotation()
                               .getRadians());
    }, 0x000000, Pose2d.class);

    new LogType((Pose3d p) -> {
      Translation3d t = p.getTranslation();
      Quaternion q = p.getRotation()
                      .getQuaternion();
      LogWriter.getInstance()
                 .writeDouble(t.getX())
                 .writeDouble(t.getY())
                 .writeDouble(t.getZ())
                 .writeDouble(q.getW())
                 .writeDouble(q.getX())
                 .writeDouble(q.getY())
                 .writeDouble(q.getZ());
    }, 0x000000, Pose3d.class);

    new LogType((Transform2d t) -> {
      Translation2d t2 = t.getTranslation();
      LogWriter.getInstance()
                 .writeDouble(t2.getX())
                 .writeDouble(t2.getY())
                 .writeDouble(t.getRotation()
                               .getRadians());
    }, 0x000000, Transform2d.class);

    new LogType((Transform3d t) -> {
      Translation3d t2 = t.getTranslation();
      Quaternion q = t.getRotation()
                      .getQuaternion();
      LogWriter.getInstance()
                 .writeDouble(t2.getX())
                 .writeDouble(t2.getY())
                 .writeDouble(t2.getZ())
                 .writeDouble(q.getW())
                 .writeDouble(q.getX())
                 .writeDouble(q.getY())
                 .writeDouble(q.getZ());
    }, 0x000000, Transform3d.class);

    new LogType((Twist2d t) -> {
      LogWriter.getInstance()
                 .writeDouble(t.dx)
                 .writeDouble(t.dy)
                 .writeDouble(t.dtheta);
    }, 0x000000, Twist2d.class);

    new LogType((Twist3d t) -> {
      LogWriter.getInstance()
                 .writeDouble(t.dx)
                 .writeDouble(t.dy)
                 .writeDouble(t.dz)
                 .writeDouble(t.rx)
                 .writeDouble(t.ry)
                 .writeDouble(t.rz);
    }, 0x000000, Twist3d.class);
  }

  private static void loadDTLib() {
    // Nothing here yet
  }
}
