package org.victorrobotics.dtlib.log;

import java.util.Objects;
import java.util.Set;
import java.util.function.BiPredicate;
import java.util.function.Consumer;

public class DTLogType {
  final Consumer    writer;
  final BiPredicate equals;
  final int         id;

  public <T> DTLogType(Consumer<T> encoder, BiPredicate<T, T> equals, int id,
      Set<Class<? extends T>> clazzes) {
    this.writer = encoder;
    this.equals = equals;
    this.id = id;
    for (Class<?> clazz : clazzes) {
      DTLogWriter.LOG_TYPES.put(clazz, this);
    }
  }

  public <T> DTLogType(Consumer<T> encoder, BiPredicate<T, T> equals, int id,
      Class<? extends T> clazz) {
    this(encoder, equals, id, Set.of(clazz));
  }

  public <T> DTLogType(Consumer<T> encoder, int id, Set<Class<? extends T>> clazzes) {
    this(encoder, Objects::equals, id, clazzes);
  }

  public <T> DTLogType(Consumer<T> encoder, int id, Class<? extends T> clazz) {
    this(encoder, Objects::equals, id, Set.of(clazz));
  }
}
