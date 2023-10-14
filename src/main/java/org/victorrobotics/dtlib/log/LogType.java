package org.victorrobotics.dtlib.log;

import java.util.Objects;
import java.util.Set;
import java.util.function.BiPredicate;
import java.util.function.Consumer;

public class LogType {
  @SuppressWarnings("rawtypes")
  final Consumer writer;

  @SuppressWarnings("rawtypes")
  final BiPredicate equals;

  final int id;

  public <T> LogType(Consumer<T> encoder, BiPredicate<T, T> equals, int id,
      Set<Class<? extends T>> clazzes) {
    this.writer = encoder;
    this.equals = equals;
    this.id = id;
    for (Class<?> clazz : clazzes) {
      LogWriter.LOG_TYPES.put(clazz, this);
    }
  }

  public <T> LogType(Consumer<T> encoder, BiPredicate<T, T> equals, int id,
      Class<? extends T> clazz) {
    this(encoder, equals, id, Set.of(clazz));
  }

  public <T> LogType(Consumer<T> encoder, int id, Set<Class<? extends T>> clazzes) {
    this(encoder, Objects::equals, id, clazzes);
  }

  public <T> LogType(Consumer<T> encoder, int id, Class<? extends T> clazz) {
    this(encoder, Objects::equals, id, Set.of(clazz));
  }
}
