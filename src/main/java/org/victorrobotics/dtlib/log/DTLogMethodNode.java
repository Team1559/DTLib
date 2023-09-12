package org.victorrobotics.dtlib.log;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

public class DTLogMethodNode extends DTLogTreeNode {
  private final Method method;

  protected DTLogMethodNode(Method method, String path) {
    super(path);
    this.method = method;
  }

  @Override
  protected Class<?> getType() {
    return method.getReturnType();
  }

  @Override
  protected Object getValue(Object parent) {
    try {
      return method.invoke(parent);
    } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
      e.printStackTrace();
      return null;
    }
  }
}
