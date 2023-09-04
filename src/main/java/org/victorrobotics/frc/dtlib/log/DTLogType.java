package org.victorrobotics.frc.dtlib.log;

public abstract class DTLogType {
  public final int       id;
  private final Class<?> clazz;

  protected DTLogType(int id, Class<?> clazz) {
    this.id = id;
    this.clazz = clazz;
    DTLogger.LOGGABLE_TYPES.put(clazz, this);
  }

  protected abstract void writeData(Object obj);

  DTLogType withClass(Class<?> clazz) {
    return new DTLogType(this.id, clazz) {
      @Override
      protected void writeData(Object obj) {
        DTLogType.this.writeData(obj);
      }
    };
  }

  protected static final DTLogWriter dataWriter() {
    return DTLogger.DATA_WRITER;
  }
}
