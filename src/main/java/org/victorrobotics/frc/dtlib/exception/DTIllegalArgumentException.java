package org.victorrobotics.frc.dtlib.exception;

import java.util.Arrays;

public class DTIllegalArgumentException extends IllegalArgumentException {
  public DTIllegalArgumentException(Object... args) {
    this(null, args);
  }

  public DTIllegalArgumentException(String message, Object... args) {
    super("Invalid argument " + new StringBuilder(Arrays.toString(args)).deleteCharAt(0)
                                                                        .reverse()
                                                                        .deleteCharAt(0)
                                                                        .reverse()
                                                                        .toString()
        + (message == null ? "" : (" because " + message)));
  }
}
