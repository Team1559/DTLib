package org.victorrobotics.dtlib.log;

import java.util.function.Supplier;

public class DTLogStaticVar extends DTLogVar {
  private final Supplier<?> accessor;

  protected DTLogStaticVar(DTLogType type, Class<?> enclosingClazz, String name, Supplier<?> accessor) {
    super(type, "static/" + enclosingClazz.getSimpleName() + "/" + name);
    this.accessor = accessor;
  }

  protected void log() {
    logValue(accessor.get());
  }
}
