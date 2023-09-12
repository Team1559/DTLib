package org.victorrobotics.dtlib.log;

import java.util.Objects;

public class DTLogVar<T> {
  private final DTLogType<T> type;
  private final String       path;

  private T   prevValue;
  private int handle;

  DTLogVar(DTLogType<T> type, String path) {
    this.type = type;
    this.path = path;
    this.handle = -1;
  }

  void logValue(T obj) {
    if (Objects.equals(prevValue, obj)) {
      return;
    }
    prevValue = obj;

    if (handle < 0) {
      handle = DTLogger.newHandle(type.id, path);
    }

    if (obj == null) {
      DTLogger.getWriter()
              .writeShort(0x0000)
              .writeShort(handle);
    } else {
      DTLogger.getWriter()
              .writeShort(handle);
      type.writer.accept(obj);
    }
  }

  @Override
  public String toString() {
    return path;
  }
}
