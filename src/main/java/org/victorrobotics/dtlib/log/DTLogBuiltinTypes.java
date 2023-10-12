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

public final class DTLogBuiltinTypes {
  private DTLogBuiltinTypes() {}

  static void load() {
    loadPrimitives();
    loadJava();
    loadWPILib();
    loadDTLib();
  }

  private static void loadPrimitives() {
    new DTLogType((Boolean b) -> {
      DTLogWriter.getInstance()
                 .writeBoolean(b.booleanValue());
    }, 0x20, Set.of(Boolean.class, boolean.class));

    new DTLogType((Byte b) -> {
      DTLogWriter.getInstance()
                 .writeByte(b.byteValue());
    }, 0x21, Set.of(Byte.class, byte.class));

    new DTLogType((Character c) -> {
      DTLogWriter.getInstance()
                 .writeChar(c.charValue());
    }, 0x22, Set.of(Character.class, char.class));

    new DTLogType((Double d) -> {
      DTLogWriter.getInstance()
                 .writeDouble(d.doubleValue());
    }, 0x23, Set.of(Double.class, double.class));

    new DTLogType((Float f) -> {
      DTLogWriter.getInstance()
                 .writeFloat(f.floatValue());
    }, 0x24, Set.of(Float.class, float.class));

    new DTLogType((Integer i) -> {
      DTLogWriter.getInstance()
                 .writeInt(i.intValue());
    }, 0x25, Set.of(Integer.class, int.class));

    new DTLogType((Long l) -> {
      DTLogWriter.getInstance()
                 .writeLong(l.longValue());
    }, 0x26, Set.of(Long.class, long.class));

    new DTLogType((Short s) -> {
      DTLogWriter.getInstance()
                 .writeShort(s.shortValue());
    }, 0x27, Set.of(Short.class, short.class));

    // 1D primitive arrays (special case)
    new DTLogType((boolean[] b) -> {
      DTLogWriter.getInstance()
                 .writeBooleanArray(b);
    }, 0x28, boolean[].class);

    new DTLogType((byte[] b) -> {
      DTLogWriter.getInstance()
                 .writeByteArray(b);
    }, 0x29, byte[].class);

    new DTLogType((char[] b) -> {
      DTLogWriter.getInstance()
                 .writeCharArray(b);
    }, 0x2A, char[].class);

    new DTLogType((double[] b) -> {
      DTLogWriter.getInstance()
                 .writeDoubleArray(b);
    }, 0x2B, double[].class);

    new DTLogType((float[] b) -> {
      DTLogWriter.getInstance()
                 .writeFloatArray(b);
    }, 0x2C, float[].class);

    new DTLogType((int[] b) -> {
      DTLogWriter.getInstance()
                 .writeIntArray(b);
    }, 0x2D, int[].class);

    new DTLogType((long[] b) -> {
      DTLogWriter.getInstance()
                 .writeLongArray(b);
    }, 0x2E, long[].class);

    new DTLogType((short[] b) -> {
      DTLogWriter.getInstance()
                 .writeShortArray(b);
    }, 0x2F, short[].class);
  }

  private static void loadJava() {
    new DTLogType((String s) -> {
      DTLogWriter.getInstance()
                 .writeStringUTF8(s);
    }, 0x30, String.class);
  }

  private static void loadWPILib() {
    new DTLogType((Rotation2d r) -> {
      DTLogWriter.getInstance()
                 .writeDouble(r.getRadians());
    }, 0x000000, Rotation2d.class);

    new DTLogType((Rotation3d r) -> {
      Quaternion q = r.getQuaternion();
      DTLogWriter.getInstance()
                 .writeDouble(q.getW())
                 .writeDouble(q.getX())
                 .writeDouble(q.getY())
                 .writeDouble(q.getZ());
    }, 0x000000, Rotation3d.class);

    new DTLogType((Translation2d t) -> {
      DTLogWriter.getInstance()
                 .writeDouble(t.getX())
                 .writeDouble(t.getY());
    }, 0x000000, Translation2d.class);

    new DTLogType((Translation3d t) -> {
      DTLogWriter.getInstance()
                 .writeDouble(t.getX())
                 .writeDouble(t.getY())
                 .writeDouble(t.getZ());
    }, 0x000000, Translation3d.class);

    new DTLogType((Pose2d p) -> {
      Translation2d t = p.getTranslation();
      DTLogWriter.getInstance()
                 .writeDouble(t.getX())
                 .writeDouble(t.getY())
                 .writeDouble(p.getRotation()
                               .getRadians());
    }, 0x000000, Pose2d.class);

    new DTLogType((Pose3d p) -> {
      Translation3d t = p.getTranslation();
      Quaternion q = p.getRotation()
                      .getQuaternion();
      DTLogWriter.getInstance()
                 .writeDouble(t.getX())
                 .writeDouble(t.getY())
                 .writeDouble(t.getZ())
                 .writeDouble(q.getW())
                 .writeDouble(q.getX())
                 .writeDouble(q.getY())
                 .writeDouble(q.getZ());
    }, 0x000000, Pose3d.class);

    new DTLogType((Transform2d t) -> {
      Translation2d t2 = t.getTranslation();
      DTLogWriter.getInstance()
                 .writeDouble(t2.getX())
                 .writeDouble(t2.getY())
                 .writeDouble(t.getRotation()
                               .getRadians());
    }, 0x000000, Transform2d.class);

    new DTLogType((Transform3d t) -> {
      Translation3d t2 = t.getTranslation();
      Quaternion q = t.getRotation()
                      .getQuaternion();
      DTLogWriter.getInstance()
                 .writeDouble(t2.getX())
                 .writeDouble(t2.getY())
                 .writeDouble(t2.getZ())
                 .writeDouble(q.getW())
                 .writeDouble(q.getX())
                 .writeDouble(q.getY())
                 .writeDouble(q.getZ());
    }, 0x000000, Transform3d.class);

    new DTLogType((Twist2d t) -> {
      DTLogWriter.getInstance()
                 .writeDouble(t.dx)
                 .writeDouble(t.dy)
                 .writeDouble(t.dtheta);
    }, 0x000000, Twist2d.class);

    new DTLogType((Twist3d t) -> {
      DTLogWriter.getInstance()
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
