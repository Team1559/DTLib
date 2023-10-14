package org.victorrobotics.dtlib;

public final class DTLibInfo {
  public static final class Version {
    public static final int    YEAR   = 0;
    public static final int    MAJOR  = 1;
    public static final int    MINOR  = 0;
    public static final String SUFFIX = null;
    public static final String STRING =
        YEAR + "." + MAJOR + "." + MINOR + (SUFFIX == null ? "" : ("-" + SUFFIX));

    private Version() {}
  }

  private DTLibInfo() {}
}
