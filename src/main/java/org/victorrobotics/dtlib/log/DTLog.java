package org.victorrobotics.dtlib.log;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Target({ ElementType.FIELD, ElementType.METHOD })
@Retention(RetentionPolicy.RUNTIME)
public @interface DTLog {
  String value() default "";

  public enum Level {
    DEBUG(0x0008),
    INFO(0x0009),
    WARN(0x000A),
    ERROR(0x000B);

    final int typeID;

    Level(int typeID) {
      this.typeID = typeID;
    }
  }
}
