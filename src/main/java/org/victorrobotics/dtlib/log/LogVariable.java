package org.victorrobotics.dtlib.log;

public class LogVariable {
  private final LogType type;
  private final String    path;

  private Object prevValue;
  private int    handle;

  LogVariable(LogType type, String path) {
    this.type = type;
    this.path = path;
    this.handle = -1;
  }

  @SuppressWarnings("unchecked")
  void logValue(Object value) {
    if (type.equals.test(prevValue, value)) return;

    if (handle < 0) {
      handle = LogWriter.getInstance()
                          .declareNewVariableHandle(type.id, path);
    }

    if (value == null) {
      LogWriter.getInstance()
                 .writeShort(0x0000)
                 .writeShort(handle);
    } else {
      LogWriter.getInstance()
                 .writeShort(handle);
      type.writer.accept(value);
    }
    prevValue = value;
  }

  @Override
  public String toString() {
    return path;
  }
}
