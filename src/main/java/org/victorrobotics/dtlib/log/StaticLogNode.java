package org.victorrobotics.dtlib.log;

import java.util.function.Supplier;

public class StaticLogNode extends LogVariable {
  private final Supplier<?> accessor;

  protected StaticLogNode(LogType type, Class<?> enclosingClazz, String name, Supplier<?> accessor) {
    super(type, "static/" + enclosingClazz.getSimpleName() + "/" + name);
    this.accessor = accessor;
  }

  protected void log() {
    logValue(accessor.get());
  }
}
