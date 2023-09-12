package org.victorrobotics.dtlib.log;

import java.lang.reflect.Field;

public class DTLogFieldNode extends DTLogTreeNode {
  private final Field field;

  protected DTLogFieldNode(Field field, String path) {
    super(path);
    this.field = field;
  }

  @Override
  protected Class<?> getType() {
    return field.getType();
  }

  @Override
  protected Object getValue(Object parent) {
    try {
      return field.get(parent);
    } catch (IllegalAccessException | IllegalArgumentException e) {
      e.printStackTrace();
      return null;
    }
  }
}
