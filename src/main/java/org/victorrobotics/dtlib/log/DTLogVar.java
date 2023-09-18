package org.victorrobotics.dtlib.log;

public class DTLogVar {
  private final DTLogType type;
  private final String    path;

  private Object prevValue;
  private int    handle;

  DTLogVar(DTLogType type, String path) {
    this.type = type;
    this.path = path;
    this.handle = -1;
  }

  @SuppressWarnings("unchecked")
  void logValue(Object value) {
    if (type.equals.test(prevValue, value)) {
      return;
    }
    prevValue = value;

    if (handle < 0) {
      handle = DTLogWriter.getInstance()
                          .declareNewVariableHandle(type.id, path);
    }

    if (value == null) {
      DTLogWriter.getInstance()
                 .writeShort(0x0000)
                 .writeShort(handle);
    } else {
      DTLogWriter.getInstance()
                 .writeShort(handle);
      type.writer.accept(value);
    }
  }

  @Override
  public String toString() {
    return path;
  }
}
