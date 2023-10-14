package org.victorrobotics.dtlib.log;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;

public class RootLogNode extends LogNode {
  private final Object           root;
  private final StaticLogNode[] staticVars;

  public RootLogNode(Object root, String name) {
    super("", name, root.getClass(), dummy -> root);
    this.root = root;

    List<StaticLogNode> staticVarList = new ArrayList<>();
    init(new ArrayDeque<>(), new LinkedHashSet<>(), staticVarList);
    staticVars = staticVarList.toArray(StaticLogNode[]::new);
  }

  public void log() {
    // Input is discarded, but cannot be null
    log(root);
    for (StaticLogNode staticVar : staticVars) {
      staticVar.log();
    }
  }
}