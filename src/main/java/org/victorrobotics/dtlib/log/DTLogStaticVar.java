package org.victorrobotics.dtlib.log;

import java.util.function.Supplier;

public class DTLogStaticVar<T> extends DTLogVar<T> {
  final Supplier<T> accessor;

  DTLogStaticVar(DTLogType<T> type, Class<?> clazz, String name, Supplier<T> accessor) {
    super(type, "static/" + clazz.getSimpleName() + "/" + name);
    this.accessor = accessor;
  }

  void log() {
    logValue(accessor.get());
  }
}
