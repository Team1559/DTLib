package org.victorrobotics.dtlib.log;

import java.util.function.Consumer;

public class DTLogType<T> {
  final Consumer<T> writer;
  final int id;

  public DTLogType(Consumer<T> encoder, int id, Class<?>... clazzes) {
    this.writer = encoder;
    this.id = id;
    for (Class<?> clazz : clazzes) {
      DTLogger.LOGGABLE_TYPES.put(clazz, this);
    }
  }
}
