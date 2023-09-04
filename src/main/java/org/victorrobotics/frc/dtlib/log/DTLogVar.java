package org.victorrobotics.frc.dtlib.log;

public class DTLogVar {
  private final DTLogType type;
  final String    path;

  private Object prevValue;
  private int    handle;

  DTLogVar(DTLogType type, String path) {
    this.type = type;
    this.path = path;
    this.handle = -1;
  }

  void logValue(Object obj) {
    if (prevValue == obj) {
      return;
    }
    prevValue = obj;

    if (handle < 0) {
      handle = DTLogger.newHandle(type.id, path);
    }

    if (obj == null) {
      DTLogger.DATA_WRITER.writeShort(0x0000)
                          .writeShort(handle);
    } else {
      DTLogger.DATA_WRITER.writeShort(handle);
      type.writeData(obj);
    }
  }
}
