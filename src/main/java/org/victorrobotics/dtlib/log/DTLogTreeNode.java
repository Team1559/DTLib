package org.victorrobotics.dtlib.log;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Deque;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

public abstract class DTLogTreeNode {
  private final String              path;
  private final List<DTLogTreeNode> children;

  @SuppressWarnings("rawtypes")
  private DTLogVar node;

  protected DTLogTreeNode(String path) {
    this.path = path;
    children = new ArrayList<>();
    node = null;
  }

  protected abstract Class<?> getType();

  protected abstract Object getValue(Object parent);

  @SuppressWarnings({ "unchecked", "rawtypes" })
  public void init(Deque<Class<?>> clazzTree, Set<Class<?>> clazzes,
      List<DTLogStaticVar<?>> staticVars) {
    Class<?> clazz = getType();
    if (clazzTree.contains(clazz)) {
      // Prevent infinite recursion
      return;
    }

    boolean includeStatic = clazzes.add(clazz);
    boolean foundValidAnnotation = false;
    while (clazz != null) {
      if (!foundValidAnnotation) {
        // Check if type is registered as directly encodeable
        DTLogType<?> type = DTLogger.LOGGABLE_TYPES.get(clazz);
        if (type != null) {
          // Terminal node
          node = new DTLogVar<>(type, path);
          return;
        }
      }

      // Check methods and fields for eligibility
      for (Field field : clazz.getDeclaredFields()) {
        DTLog annotation = field.getAnnotation(DTLog.class);
        if (annotation == null) {
          continue;
        }

        boolean isStatic = Modifier.isStatic(field.getModifiers());
        if (isStatic && !includeStatic) {
          continue;
        }

        if (!field.trySetAccessible()) {
          continue;
        }

        String name = annotation.value();
        if (!isValidName(name)) {
          name = field.getName();
        }

        if (isStatic) {
          DTLogType<?> type = DTLogger.LOGGABLE_TYPES.get(field.getType());
          if (type != null) {
            staticVars.add(new DTLogStaticVar(type, field.getDeclaringClass(), name, () -> {
              try {
                return field.get(null);
              } catch (IllegalAccessException | IllegalArgumentException e) {
                e.printStackTrace();
                return null;
              }
            }));
          }
        } else {
          foundValidAnnotation = true;
          children.add(new DTLogFieldNode(field, path + "/" + name));
        }
      }

      for (Method method : clazz.getDeclaredMethods()) {
        DTLog annotation = method.getAnnotation(DTLog.class);
        if (annotation == null) {
          continue;
        }

        if (method.getParameterCount() != 0 || method.getReturnType() == void.class) {
          // Not a getter method
          continue;
        }

        boolean isStatic = Modifier.isStatic(method.getModifiers());
        if (isStatic && !includeStatic) {
          continue;
        }

        if (!method.trySetAccessible()) {
          continue;
        }

        String name = annotation.value();
        if (!isValidName(name)) {
          name = method.getName() + "()";
        }

        if (isStatic) {
          DTLogType<?> type = DTLogger.LOGGABLE_TYPES.get(method.getReturnType());
          if (type != null) {
            staticVars.add(new DTLogStaticVar(type, method.getDeclaringClass(), name, () -> {
              try {
                return method.invoke(null);
              } catch (IllegalAccessException | IllegalArgumentException
                  | InvocationTargetException e) {
                e.printStackTrace();
                return null;
              }
            }));
          }
        } else {
          foundValidAnnotation = true;
          children.add(new DTLogMethodNode(method, path + "/" + name));
        }
      }

      clazz = clazz.getSuperclass();
    }

    // Init children
    clazzTree.addFirst(getType());
    for (DTLogTreeNode child : children) {
      child.init(clazzTree, clazzes, staticVars);
    }
    clazzTree.removeFirst();

    // Prune empty branches
    Iterator<DTLogTreeNode> itr = children.iterator();
    while (itr.hasNext()) {
      DTLogTreeNode child = itr.next();
      if (child.node == null && child.children.isEmpty()) {
        itr.remove();
      }
    }
  }

  @SuppressWarnings("unchecked")
  void log(Object parent) {
    if (parent == null) {
      logNull();
      return;
    }

    Object me = getValue(parent);
    if (node != null) {
      node.logValue(me);
    } else {
      for (DTLogTreeNode child : children) {
        child.log(me);
      }
    }
  }

  @SuppressWarnings("unchecked")
  private void logNull() {
    if (node != null) {
      node.logValue(null);
    } else {
      for (DTLogTreeNode child : children) {
        child.logNull();
      }
    }
  }

  @Override
  public String toString() {
    if (node != null) {
      return node.toString();
    } else {
      return children.stream()
                     .map(DTLogTreeNode::toString)
                     .reduce("", (s1, s2) -> s1 + "\n" + s2);
    }
  }

  private static boolean isValidName(String name) {
    return !"".equals(name) && name.indexOf('/') == -1;
  }
}
