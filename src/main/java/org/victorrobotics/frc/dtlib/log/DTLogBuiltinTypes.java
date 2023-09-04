package org.victorrobotics.frc.dtlib.log;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Random;
import java.util.function.Supplier;
import java.util.function.ToLongFunction;

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
    new DTLogType(0x0020, Boolean.class) {
      @Override
      protected void writeData(Object obj) {
        dataWriter().writeBoolean(((Boolean) obj).booleanValue());
      }
    }.withClass(boolean.class);
    new DTLogType(0x0021, Byte.class) {
      @Override
      protected void writeData(Object obj) {
        dataWriter().writeByte(((Byte) obj).byteValue());
      }
    }.withClass(byte.class);
    new DTLogType(0x0022, Character.class) {
      @Override
      protected void writeData(Object obj) {
        dataWriter().writeChar(((Character) obj).charValue());
      }
    }.withClass(char.class);
    new DTLogType(0x0023, Double.class) {
      @Override
      protected void writeData(Object obj) {
        dataWriter().writeDouble(((Double) obj).doubleValue());
      }
    }.withClass(double.class);
    new DTLogType(0x0024, Float.class) {
      @Override
      protected void writeData(Object obj) {
        dataWriter().writeFloat(((Float) obj).floatValue());
      }
    }.withClass(float.class);
    new DTLogType(0x0025, Integer.class) {
      @Override
      protected void writeData(Object obj) {
        dataWriter().writeInt(((Integer) obj).intValue());
      }
    }.withClass(int.class);
    new DTLogType(0x0026, Long.class) {
      @Override
      protected void writeData(Object obj) {
        dataWriter().writeLong(((Long) obj).longValue());
      }
    }.withClass(long.class);
    new DTLogType(0x0027, Short.class) {
      @Override
      protected void writeData(Object obj) {
        dataWriter().writeShort(((Short) obj).shortValue());
      }
    }.withClass(short.class);
    // 1D Arrays
    new DTLogType(0x0028, boolean[].class) {
      @Override
      protected void writeData(Object obj) {
        dataWriter().writeBooleanArray((boolean[]) obj);
      }
    };
    new DTLogType(0x0029, byte[].class) {
      @Override
      protected void writeData(Object obj) {
        dataWriter().writeByteArray((byte[]) obj);
      }
    };
    new DTLogType(0x002a, char[].class) {
      @Override
      protected void writeData(Object obj) {
        dataWriter().writeCharArray((char[]) obj);
      }
    };
    new DTLogType(0x002b, double[].class) {
      @Override
      protected void writeData(Object obj) {
        dataWriter().writeDoubleArray((double[]) obj);
      }
    };
    new DTLogType(0x002c, float[].class) {
      @Override
      protected void writeData(Object obj) {
        dataWriter().writeFloatArray((float[]) obj);
      }
    };
    new DTLogType(0x002d, int[].class) {
      @Override
      protected void writeData(Object obj) {
        dataWriter().writeIntArray((int[]) obj);
      }
    };
    new DTLogType(0x002e, long[].class) {
      @Override
      protected void writeData(Object obj) {
        dataWriter().writeLongArray((long[]) obj);
      }
    };
    new DTLogType(0x002f, short[].class) {
      @Override
      protected void writeData(Object obj) {
        dataWriter().writeShortArray((short[]) obj);
      }
    };
  }

  private static void loadJava() {
    new DTLogType(0x0030, String.class) {
      @Override
      protected void writeData(Object obj) {
        dataWriter().writeStringUTF8((String) obj);
      }
    };
  }

  private static void loadWPILib() {
    new DTLogType(0x00000000, Rotation2d.class) {
      @Override
      protected void writeData(Object obj) {
        dataWriter().writeDouble(((Rotation2d) obj).getRadians());
      }
    };
    new DTLogType(0x00000000, Rotation3d.class) {
      @Override
      protected void writeData(Object obj) {
        Quaternion q = ((Rotation3d) obj).getQuaternion();
        dataWriter().writeDouble(q.getW())
                    .writeDouble(q.getX())
                    .writeDouble(q.getY())
                    .writeDouble(q.getZ());
      }
    };
    new DTLogType(0x00000000, Translation2d.class) {
      @Override
      protected void writeData(Object obj) {
        Translation2d t = (Translation2d) obj;
        dataWriter().writeDouble(t.getX())
                    .writeDouble(t.getY());
      }
    };
    new DTLogType(0x00000000, Translation3d.class) {
      @Override
      protected void writeData(Object obj) {
        Translation3d t = (Translation3d) obj;
        dataWriter().writeDouble(t.getX())
                    .writeDouble(t.getY())
                    .writeDouble(t.getZ());
      }
    };
    new DTLogType(0x00000000, Pose2d.class) {
      @Override
      protected void writeData(Object obj) {
        Pose2d p = (Pose2d) obj;
        Translation2d t = p.getTranslation();
        dataWriter().writeDouble(t.getX())
                    .writeDouble(t.getY())
                    .writeDouble(p.getRotation()
                                  .getRadians());
      }
    };
    new DTLogType(0x00000000, Pose3d.class) {
      @Override
      protected void writeData(Object obj) {
        Pose3d p = (Pose3d) obj;
        Translation3d t = p.getTranslation();
        Quaternion q = p.getRotation()
                        .getQuaternion();
        dataWriter().writeDouble(t.getX())
                    .writeDouble(t.getY())
                    .writeDouble(t.getZ())
                    .writeDouble(q.getW())
                    .writeDouble(q.getX())
                    .writeDouble(q.getY())
                    .writeDouble(q.getZ());
      }
    };
    new DTLogType(0x00000000, Transform2d.class) {
      @Override
      protected void writeData(Object obj) {
        Transform2d t = (Transform2d) obj;
        Translation2d t2 = t.getTranslation();
        dataWriter().writeDouble(t2.getX())
                    .writeDouble(t2.getY())
                    .writeDouble(t.getRotation()
                                  .getRadians());
      }
    };
    new DTLogType(0x00000000, Transform3d.class) {
      @Override
      protected void writeData(Object obj) {
        Transform3d t = (Transform3d) obj;
        Translation3d t2 = t.getTranslation();
        Quaternion q = t.getRotation()
                        .getQuaternion();
        dataWriter().writeDouble(t2.getX())
                    .writeDouble(t2.getY())
                    .writeDouble(t2.getZ())
                    .writeDouble(q.getW())
                    .writeDouble(q.getX())
                    .writeDouble(q.getY())
                    .writeDouble(q.getZ());
      }
    };
    new DTLogType(0x00000000, Twist2d.class) {
      @Override
      protected void writeData(Object obj) {
        Twist2d t = (Twist2d) obj;
        dataWriter().writeDouble(t.dx)
                    .writeDouble(t.dy)
                    .writeDouble(t.dtheta);
      }
    };
    new DTLogType(0x00000000, Twist3d.class) {
      @Override
      protected void writeData(Object obj) {
        Twist3d t = (Twist3d) obj;
        dataWriter().writeDouble(t.dx)
                    .writeDouble(t.dy)
                    .writeDouble(t.dz)
                    .writeDouble(t.rx)
                    .writeDouble(t.ry)
                    .writeDouble(t.rz);
      }
    };
  }

  private static void loadDTLib() {}
}
