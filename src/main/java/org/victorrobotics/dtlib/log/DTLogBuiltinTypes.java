package org.victorrobotics.dtlib.log;

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

public class DTLogBuiltinTypes {
  private DTLogBuiltinTypes() {}

  static void load() {
    loadPrimitives();
    loadJava();
    loadWPILib();
    loadDTLib();
  }

  private static void loadPrimitives() {
    new DTLogType<>((Boolean b) -> {
      DTLogger.getWriter()
              .writeBoolean(b.booleanValue());
    }, 0x20, Boolean.class, boolean.class);

    new DTLogType<>((Byte b) -> {
      DTLogger.getWriter()
              .writeByte(b.byteValue());
    }, 0x21, Byte.class, byte.class);

    new DTLogType<>((Character c) -> {
      DTLogger.getWriter()
              .writeChar(c.charValue());
    }, 0x22, Character.class, char.class);

    new DTLogType<>((Double d) -> {
      DTLogger.getWriter()
              .writeDouble(d.doubleValue());
    }, 0x23, Double.class, double.class);

    new DTLogType<>((Float f) -> {
      DTLogger.getWriter()
              .writeFloat(f.floatValue());
    }, 0x24, Float.class, float.class);

    new DTLogType<>((Integer i) -> {
      DTLogger.getWriter()
              .writeInt(i.intValue());
    }, 0x25, Integer.class, int.class);

    new DTLogType<>((Long l) -> {
      DTLogger.getWriter()
              .writeLong(l.longValue());
    }, 0x26, Long.class, long.class);

    new DTLogType<>((Short s) -> {
      DTLogger.getWriter()
              .writeShort(s.shortValue());
    }, 0x27, Short.class, short.class);

    // 1D primitive arrays (special case)
    new DTLogType<>((boolean[] b) -> {
      DTLogger.getWriter()
              .writeBooleanArray(b);
    }, 0x28, boolean[].class);

    new DTLogType<>((byte[] b) -> {
      DTLogger.getWriter()
              .writeByteArray(b);
    }, 0x29, byte[].class);

    new DTLogType<>((char[] b) -> {
      DTLogger.getWriter()
              .writeCharArray(b);
    }, 0x2a, char[].class);

    new DTLogType<>((double[] b) -> {
      DTLogger.getWriter()
              .writeDoubleArray(b);
    }, 0x2b, double[].class);

    new DTLogType<>((float[] b) -> {
      DTLogger.getWriter()
              .writeFloatArray(b);
    }, 0x2c, float[].class);

    new DTLogType<>((int[] b) -> {
      DTLogger.getWriter()
              .writeIntArray(b);
    }, 0x2d, int[].class);

    new DTLogType<>((long[] b) -> {
      DTLogger.getWriter()
              .writeLongArray(b);
    }, 0x2e, long[].class);

    new DTLogType<>((short[] b) -> {
      DTLogger.getWriter()
              .writeShortArray(b);
    }, 0x2f, short[].class);
  }

  private static void loadJava() {
    new DTLogType<>((String s) -> {
      DTLogger.getWriter()
              .writeStringUTF8(s);
    }, 0x30, String.class);
  }

  private static void loadWPILib() {
    new DTLogType<>((Rotation2d r) -> {
      DTLogger.getWriter()
              .writeDouble(r.getRadians());
    }, 0x000000, Rotation2d.class);

    new DTLogType<>((Rotation3d r) -> {
      Quaternion q = r.getQuaternion();
      DTLogger.getWriter()
              .writeDouble(q.getW())
              .writeDouble(q.getX())
              .writeDouble(q.getY())
              .writeDouble(q.getZ());
    }, 0x000000, Rotation3d.class);

    new DTLogType<>((Translation2d t) -> {
      DTLogger.getWriter()
              .writeDouble(t.getX())
              .writeDouble(t.getY());
    }, 0x000000, Translation2d.class);

    new DTLogType<>((Translation3d t) -> {
      DTLogger.getWriter()
              .writeDouble(t.getX())
              .writeDouble(t.getY())
              .writeDouble(t.getZ());
    }, 0x000000, Translation3d.class);

    new DTLogType<>((Pose2d p) -> {
      Translation2d t = p.getTranslation();
      DTLogger.getWriter()
              .writeDouble(t.getX())
              .writeDouble(t.getY())
              .writeDouble(p.getRotation()
                            .getRadians());
    }, 0x000000, Pose2d.class);

    new DTLogType<>((Pose3d p) -> {
      Translation3d t = p.getTranslation();
      Quaternion q = p.getRotation()
                      .getQuaternion();
      DTLogger.getWriter()
              .writeDouble(t.getX())
              .writeDouble(t.getY())
              .writeDouble(t.getZ())
              .writeDouble(q.getW())
              .writeDouble(q.getX())
              .writeDouble(q.getY())
              .writeDouble(q.getZ());
    }, 0x000000, Pose3d.class);

    new DTLogType<>((Transform2d t) -> {
      Translation2d t2 = t.getTranslation();
      DTLogger.getWriter()
              .writeDouble(t2.getX())
              .writeDouble(t2.getY())
              .writeDouble(t.getRotation()
                            .getRadians());
    }, 0x000000, Transform2d.class);

    new DTLogType<>((Transform3d t) -> {
      Translation3d t2 = t.getTranslation();
      Quaternion q = t.getRotation()
                      .getQuaternion();
      DTLogger.getWriter()
              .writeDouble(t2.getX())
              .writeDouble(t2.getY())
              .writeDouble(t2.getZ())
              .writeDouble(q.getW())
              .writeDouble(q.getX())
              .writeDouble(q.getY())
              .writeDouble(q.getZ());
    }, 0x000000, Transform3d.class);

    new DTLogType<>((Twist2d t) -> {
      DTLogger.getWriter()
              .writeDouble(t.dx)
              .writeDouble(t.dy)
              .writeDouble(t.dtheta);
    }, 0x000000, Twist2d.class);

    new DTLogType<>((Twist3d t) -> {
      DTLogger.getWriter()
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
