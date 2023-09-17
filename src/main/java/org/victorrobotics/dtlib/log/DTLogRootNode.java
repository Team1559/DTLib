package org.victorrobotics.dtlib.log;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;

public class DTLogRootNode extends DTLogTreeNode {
  private final Object           root;
  private final DTLogStaticVar[] staticVars;

  public DTLogRootNode(Object root, String name) {
    super("", name, root.getClass(), dummy -> root);
    this.root = root;

    List<DTLogStaticVar> staticVarList = new ArrayList<>();
    init(new ArrayDeque<>(), new LinkedHashSet<>(), staticVarList);
    staticVars = staticVarList.toArray(DTLogStaticVar[]::new);
  }

  public void log() {
    // Input is discarded, but cannot be null
    log(root);
    for (DTLogStaticVar staticVar : staticVars) {
      staticVar.log();
    }
  }
}
