package org.victorrobotics.dtlib.log;

import java.util.ArrayDeque;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;

public class RootLogNode extends LogNode {
  private static final Object NO_PARENT_OBJ = new Object();

  private final StaticLogVariable[] staticVars;

  public RootLogNode(Object robot, DTLog.Level robotLogLevel) {
    super("", robot.toString(), robot.getClass(), unused -> robot);

    Map<StaticLogVariable, DTLog.Level> staticVarList = new LinkedHashMap<>();
    init(new ArrayDeque<>(), new LinkedHashSet<>(), staticVarList, robotLogLevel);

    staticVarList.entrySet()
                 .removeIf(entry -> {
                   DTLog.Level varLogLevel = entry.getValue();
                   return varLogLevel.ordinal() < robotLogLevel.ordinal();
                 });
    staticVars = staticVarList.keySet()
                              .toArray(StaticLogVariable[]::new);
  }

  public void log() {
    log(NO_PARENT_OBJ);
    for (StaticLogVariable staticVar : staticVars) {
      staticVar.log();
    }
  }
}
