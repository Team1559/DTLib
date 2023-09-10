package org.victorrobotics.dtlib.exception;

import java.util.Objects;

public class DTIllegalArgumentException extends IllegalArgumentException {
  public DTIllegalArgumentException(Object arg) {
    super("Invalid argument " + Objects.toString(arg));
  }

  public DTIllegalArgumentException(Object arg, String message) {
    super("Invalid argument " + Objects.toString(arg) + " because " + message);
  }
}
