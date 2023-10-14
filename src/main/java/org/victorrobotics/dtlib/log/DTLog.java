package org.victorrobotics.dtlib.log;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * An annotation that indicates to DTLog that an element's value changes should
 * be recorded every robot cycle. This annotation can be applied in several
 * different places, including object members, object getter methods, static
 * class variables and constants, and static getter methods. However, this
 * annotation alone does not guarantee discovery by the logger.
 *
 * @see <a href=
 *      "https://github.com/Team1559/DTLib/blob/master/src/main/java/org/victorrobotics/dtlib/log/DTLog.md">DTLog.md</a>
 */
@Target({ ElementType.FIELD, ElementType.METHOD })
@Retention(RetentionPolicy.RUNTIME)
public @interface DTLog {
  /**
   * The name to use in the variable's logging path (and its children). If empty
   * (default), it will be automatically generated from the accessor's
   * properties.
   *
   * @return the loggable name of this variable
   */
  String name() default "";

  /**
   * The minimum level at which to begin logging this variable. If this level is
   * more verbose than this variable's parent object, the variable will be
   * ignored.
   *
   * @return
   */
  Level level() default Level.INFO;

  /**
   * The amount of detail to include in generated logs and messages.
   * <p>
   * Robots: a global setting. Variables and messages more verbose than this
   * will be ignored. Defaults to {@link #INFO}.
   * <p>
   * Variables: determines whether a variable (and its children, if any) will be
   * logged. Variables more verbose than the log level of their parent will be
   * ignored. Defaults to {@link #INFO}.
   * <p>
   * Messages: determines whether a text message will be logged to disk, AND
   * whether it will be printed. Messages more verbose than the robot log level
   * will be ignored. Defaults to {@link #INFO}.
   */
  public enum Level {
    /**
     * The most verbose level, recording all data tagged as loggable.
     * <p>
     * Robots: use only while debugging specific code. Should not be used during
     * competition, as it can incur a significant performance penalty. Enable
     * one subsystem at most to prevent clutter.
     * <p>
     * Variables: use when obtaining values that may incur a performance penalty
     * (e.g. cacheless CAN transactions), or values that don't seem useful most
     * of the time.
     * <p>
     * Messages: use for large nonessential messages and traces.
     */
    DEBUG(0x0008),
    /**
     * A level that provides useful information in addition to warnings and
     * errors. This is the default value for all categories.
     * <p>
     * Robots: use during general development. Not reecommended during
     * competition due to cluttered console output.
     * <p>
     * Variables: use for general logging of data points.
     * <p>
     * Messages: use for succinct nonessential messages.
     */
    INFO(0x0009),
    /**
     * A level that provides a balance between data and performance/bandwidth.
     * <p>
     * Robots: use during testing. Recommended for use during competition.
     * <p>
     * Variables: use for variables that should be logged consistently.
     * <p>
     * Messages: use for warnings that aren't critical, but should be fixed.
     * Text will be bolded in yellow.
     */
    WARN(0x000A),
    /**
     * A level that uses as little bandwidth as possible, logging only essential
     * data and messages.
     * <p>
     * Robots: not recommended unless performance becomes an issue.
     * <p>
     * Variables: use for essential variables that must always be logged.
     * <p>
     * Messages: use for warnings which require immediate attention. Text will
     * be bolded in red.
     */
    ERROR(0x000B);

    final int typeID;

    Level(int typeID) {
      this.typeID = typeID;
    }
  }
}
