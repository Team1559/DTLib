package org.victorrobotics.dtlib.log;

import org.victorrobotics.dtlib.DTRobot;

import java.util.ArrayDeque;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;

public class DTLogRootNode extends DTLogTreeNode {
  private static final Object NO_PARENT_OBJ = new Object();

  private final DTLogStaticVar[] staticVars;

  public DTLogRootNode(DTRobot robot, DTLog.Level robotLogLevel) {
    super("", robot.getName(), robot.getClass(), unused -> robot);

    Map<DTLogStaticVar, DTLog.Level> staticVarList = new LinkedHashMap<>();
    init(new ArrayDeque<>(), new LinkedHashSet<>(), staticVarList, robotLogLevel);

    staticVarList.entrySet()
                 .removeIf(entry -> {
                   DTLog.Level varLogLevel = entry.getValue();
                   return varLogLevel.ordinal() < robotLogLevel.ordinal();
                 });
    staticVars = staticVarList.keySet()
                              .toArray(DTLogStaticVar[]::new);
  }

  public void log() {
    log(NO_PARENT_OBJ);
    for (DTLogStaticVar staticVar : staticVars) {
      staticVar.log();
    }
  }
}
