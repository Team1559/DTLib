package org.victorrobotics.dtlib.log;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;

public class DTLogRootNode extends DTLogTreeNode {
  private final Object               robot;
  private final Class<?>             clazz;
  private final List<DTLogStaticVar<?>> staticVars;

  public DTLogRootNode(Object robot) {
    super(robot.getClass()
               .getSimpleName());
    this.robot = robot;
    this.clazz = robot.getClass();
    this.staticVars = new ArrayList<>();

    init(new ArrayDeque<>(), new LinkedHashSet<>(), staticVars);
  }

  @Override
  protected Class<?> getType() {
    return clazz;
  }

  @Override
  protected Object getValue(Object parent) {
    return robot;
  }

  public void log() {
    // Input is discarded, but cannot be null
    log(robot);
    for (DTLogStaticVar<?> v : staticVars) {
      v.log();
    }
  }
}
