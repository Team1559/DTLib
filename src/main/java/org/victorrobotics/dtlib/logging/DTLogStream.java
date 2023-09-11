package org.victorrobotics.dtlib.logging;

import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.TreeSet;

public class DTLogStream {
  private static final Set<Integer> USED_UNIQUE_IDS = new TreeSet<>(List.of(-1));
  private static final Random       RANDOM          = new Random();

  private final int         uniqueID;
  private final DTLogFormat format;

  public DTLogStream(DTLogFormat format) {
    this.format = format;
    uniqueID = generateUniqueID();
  }

  private static int generateUniqueID() {
    synchronized (USED_UNIQUE_IDS) {
      int id = -1;
      while (USED_UNIQUE_IDS.contains(id)) {
        id = RANDOM.nextInt(1 << 30);
      }
      USED_UNIQUE_IDS.add(id);
      return id;
    }
  }
}
