# DTLog

A data-logging framework, driven by Java reflection and annotations, with support for custom data types and microsecond timestamp resolution.

## Usage

In order to use DTLog, the robot class must extend `DTRobot`. Once the robot is started, DTLog will automatically initialize and attempt to search the robot for variables to log.

### Variables

To declare a variable of any visibility as loggable, simply annotate it with `@DTLog`:

```java
public class Robot extends DTRobot {
  @DTLog
  private double vx;
}
```

This creates a `double` variable with path "`Robot/vx`". We can override the logged name of the variable by supplying it to the annotation:

```java
public class Robot extends DTRobot {
  @DTLog("velocityX")
  private double vx;
}
```

The variable path is now "`Robot/velocityX`". This tag can also be applied to simple getter methods, which don't take any parameters:

```java
public class Robot extends DTRobot {
  private double vx;
  private double vy;

  @DTLog("velocity")
  private double getVelocity() {
    return Math.hypot(vx, vy);
  }
}
```

Finally, static getter methods and variables are also supported:

```java
public class Robot extends DTRobot {
  @DTLog("velocityMaximum")
  private static final double MAX_VELOCITY = 5.0;

  @DTLog
  private static double getMaxRotation() {
    return 2 * Math.PI;
  }
}
```

These have the paths "`static/Robot/velocityMaximum`" and "`static/Robot/getMaxRotation()`", respectively

### Subcomponents

Robots often contain custom user-written subsystems that aren't natively supported by DTLog. These can be logged by declaring them as loggable within their class, then declaring the enclosing variable as loggable:

```java
public class DriveTrain {
  @DTLog
  private double velocityX;

  @DTLog
  private double velocityY;
}
...
public class Robot extends DTRobot {
  @DTLog
  private DriveTrain driveTrain;
}
```

### Custom Data Types

For some immutible data types, it doesn't make sense to give individual variables separate logging entries. For example, take a data class that stores a point on an autonomous route:

```java
public class RoutePoint {
  public final long   time;
  public final double xPos;
  public final double yPos;
  public final double xVel;
  public final double yVel;
}
```

Using the previous component strategy, this would create five new variables per  data point. This is incredibly inefficient. Instead, we can provide DTLog with a special serialization by constructing a new `DTLogType` before starting the robot. It must also be assigned a valid custom typeID (see the specification below).

```java
new DTLogType<>((RoutePoint point) -> {
  DTLog.getWriter()
          .writeLong(point.time)
          .writeDouble(point.xPos)
          .writeDouble(point.yPos)
          .writeDouble(point.xVel)
          .writeDouble(point.yVel);
}, 0x80, RoutePoint.class);
```

We don't need to save a reference to this object ourselves: upon construction, it is automatically registered by DTLog, and the type can now be logged as one consolidated variable rather than five separate ones.

This is intended for immutible objects, as DTLog can only detect when the object reference changes. To detect different objects with the same value and prevent logging redundant data, it is also recommended to implement `equals()`:

```java
public class RoutePoint {
  ...
  @Override
  public boolean equals(RoutePoint other) {
    return other != null && time == other.time && xPos == other.xPos
        && yPos == other.yPos && xVel == other.xVel && yVel == other.yVel;
  }
}
```

... or to provide another implementation for DTLog upon constructing the `DTLogType`:

```java
new DTLogType<>((RoutePoint point) -> {
  ...
}, (RoutePoint p1, RoutePoint p2) -> {
  if (p1 == p2) {
    return true;
  } else if (p1 == null || p2 == null) {
    return false;
  }
  return p1.time == p2.time && p1.xPos == p2.xPos && p1.yPos == p2.yPos
      && p1.xVel == p2.xVel && p1.yVel == p2.yVel;
}, 0x80, RoutePoint.class);
```

## Log File Specification

The following defines the format of the files output by DTLog, for the purpose of implementing compatible readers and writers. A couple of notes and definitions:

- Log files have the extension ".dtlog"
- Byte ordering (for integers, doubles, etc.) is **big endian**
- All timestamps are in **milliseconds**
- A **UTF_STR** is a string of characters, encoded into its UTF-8 representation, prepended by a two-byte string length (in bytes). The length is limited to a maximum of **65535 bytes**.

### Header

Each log file begins with a 32-byte header containing metadata about the log. This header is as follows:

1. Magic String
    - The 12-byte string "DTLib Logger" in UTF-8 (no length prepended)
2. DTLib Version
    - 2 bytes for the year
    - 1 byte for the major version
    - 1 byte for the minor version
3. WPILib Version
    - 2 bytes for the year
    - 1 byte for the major version
    - 1 byte for the minor version
4. Team Number
    - 2 bytes for the team number, or `0xFFFF` if unknown
5. Timestamp
    - A 6-byte integer of the current system time since the Unix epoch in milliseconds
6. Checksum
    - Equal to XORing every 4-byte sequence in the previous 28 bytes of metadata
    - To check file validity, a log reader XORs all 32 bytes in 4-byte increments, and the result should be 0

Sample header:

```java
// Entire header, split into 4-byte sequences
0x44544C69_62204C6F_67676572_07E70001_07E70403_0617018A_8F464F95_C8422F69

// "DTLib Logger"
0x44544C69_62204C6F_67676572

// DTLib 2023.0.1
0x07E70001

// WPILib 2023.4.3
0x07E70403

// Team 1559
0x0617

// Wed Sep 13 2023 16:01:04
0x018A_8F464F95

// Checksum
0xC8422F69
```

The remainder of the log file is a stream of binary data, encoded as identifiers followed by arguments. Therefore, the file must be loaded from its beginning, but not necessarily to its end, to successfully decode the data it contains.

### Identifiers

Identifiers are any of the 65536 possible 2-byte sequences, which are grouped into three categories:

- `0x0000` through `0x001F`: reserved for special commands
    - `0x0000` - NO_VALUE
        - Sets a variable to a null state
        - Argument: the 2-byte variable handle
    - `0x0001` - TIME_INC
        - Increments the timestamp for future data records
        - Argument: a 2-byte time increment in milliseconds
    - `0x0002` - TIME_SET
        - Sets the timestamp for future data records
        - Argument: a 6-byte timestamp in milliseconds
    - `0x0003` - ESTOP
        - Declares that the robot has been emergency-stopped
        - Argument: none
    - `0x0004` through `0x0007` - MODE_SET
        - Declares that the robot's current mode has been changed
        - New mode determined by identifier
            - `0x0004` = DISABLE
            - `0x0005` = TEST
            - `0x0006` = TELEOP
            - `0x0007` = AUTO
        - Argument: none
    - `0x0008` through `0x000B` - LOG_MSG
        - Logs a message into the file
        - Log level determined by identifier
            - `0x0008` = DEBUG
            - `0x0009` = INFO
            - `0x000A` = WARN
            - `0x000B` = ERROR
        - Argument: a UTF_STR, the message
    - `0x000C` through `0x001F` - reserved for future use
- `0x0020` through `0x00FF`: new variables
    - Declares a variable with the type corresponding to the identifier, and assigns it the next available handle (in ascending order)
    - `0x0020` through `0x007F` are reserved for types built into DTLog (up to 96)
    - `0x0080` through `0x00FF` may be used for custom data types (up to 128)
    - Argument: a UTF_STR, the variable path
- `0x0100` through `0xFFFF`: variable handles
    - Changes the value of the corresponding variable assigned
    - Argument: the new encoded value of the variable, as defined by the associated `DTLogType`
