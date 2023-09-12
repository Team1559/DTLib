package org.victorrobotics.dtlib.logging;

import java.io.DataOutputStream;

public interface DTLoggable {
  void log(DataOutputStream dataStream);

  void skeleton();
}
