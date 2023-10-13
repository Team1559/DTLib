package org.victorrobotics.dtlib.log;

import static org.victorrobotics.dtlib.log.DTLogWriter.LOG_PATH_SEPARATOR;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Deque;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.UnaryOperator;

public class DTLogTreeNode {
  private final String                path;
  private final Class<?>              type;
  private final UnaryOperator<Object> getter;

  private DTLogTreeNode[] children;
  private DTLogVar        variable;

  public DTLogTreeNode(String path, String name, Class<?> type, UnaryOperator<Object> getter) {
    this.path = path + LOG_PATH_SEPARATOR + name;
    this.type = type;
    this.getter = getter;
  }

  protected final void init(Deque<Class<?>> stack, Set<Class<?>> clazzes,
                            Map<DTLogStaticVar, DTLog.Level> staticVars, DTLog.Level logLevel) {
    if (stack.contains(type)) {
      // Prevent infinite recursion
      return;
    }

    Map<DTLogTreeNode, DTLog.Level> childrenMap = new LinkedHashMap<>();
    Class<?> clazz = type;
    while (clazz != null) {
      if (childrenMap.isEmpty()) {
        DTLogType logType = DTLogWriter.LOG_TYPES.get(clazz);
        if (logType != null) {
          variable = new DTLogVar(logType, path);
          return;
        }
      }

      boolean includeStatic = clazzes.add(clazz);

      for (Field field : clazz.getDeclaredFields()) {
        initField(field, childrenMap, staticVars, includeStatic);
      }

      for (Method method : clazz.getDeclaredMethods()) {
        initMethod(method, childrenMap, staticVars, includeStatic);
      }

      clazz = clazz.getSuperclass();
    }

    stack.addFirst(type);
    childrenMap.entrySet()
               .removeIf(entry -> {
                 DTLog.Level childLogLevel = entry.getValue();
                 if (childLogLevel.ordinal() < logLevel.ordinal()) return true;

                 DTLogTreeNode child = entry.getKey();
                 child.init(stack, clazzes, staticVars, childLogLevel);
                 return child.variable == null && child.children == null;
               });
    stack.removeFirst();

    this.children = childrenMap.isEmpty() ? null : childrenMap.keySet()
                                                              .toArray(DTLogTreeNode[]::new);
  }

  private void initField(Field field, Map<DTLogTreeNode, DTLog.Level> childList,
                         Map<DTLogStaticVar, DTLog.Level> staticVars, boolean includeStatic) {
    DTLog annotation = field.getAnnotation(DTLog.class);
    if (annotation == null) return;

    boolean isStatic = Modifier.isStatic(field.getModifiers());
    if (isStatic && !includeStatic) return;

    if (!field.trySetAccessible()) return;

    String name = annotation.name();
    if (!isValidName(name)) {
      name = field.getName();
    }

    if (isStatic) {
      DTLogType logType = DTLogWriter.LOG_TYPES.get(field.getType());
      if (logType == null) return;

      staticVars.put(new DTLogStaticVar(logType, field.getDeclaringClass(), name, () -> {
        try {
          return field.get(null);
        } catch (IllegalAccessException | IllegalArgumentException e) {
          return null;
        }
      }), annotation.level());
      return;
    }

    childList.put(new DTLogTreeNode(path, name, field.getType(), parent -> {
      try {
        return field.get(parent);
      } catch (IllegalAccessException | IllegalArgumentException e) {
        return null;
      }
    }), annotation.level());
  }

  private void initMethod(Method method, Map<DTLogTreeNode, DTLog.Level> childList,
                          Map<DTLogStaticVar, DTLog.Level> staticVars, boolean includeStatic) {
    DTLog annotation = method.getAnnotation(DTLog.class);
    if (annotation == null) return;

    if (method.getParameterCount() != 0 || method.getReturnType() == void.class) return;

    boolean isStatic = Modifier.isStatic(method.getModifiers());
    if (isStatic && !includeStatic) return;

    if (!method.trySetAccessible()) return;

    String name = annotation.name();
    if (!isValidName(name)) {
      name = method.getName() + "()";
    }

    if (isStatic) {
      DTLogType logType = DTLogWriter.LOG_TYPES.get(method.getReturnType());
      if (logType == null) return;

      staticVars.put(new DTLogStaticVar(logType, method.getDeclaringClass(), name, () -> {
        try {
          return method.invoke(null, (Object[]) null);
        } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
          return null;
        }
      }), annotation.level());
      return;
    }

    childList.put(new DTLogTreeNode(path, name, method.getReturnType(), parent -> {
      try {
        return method.invoke(parent, (Object[]) null);
      } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
        return null;
      }
    }), annotation.level());
  }

  protected void log(Object parent) {
    if (parent == null) {
      logNull();
      return;
    }

    Object value = getter.apply(parent);
    if (variable != null) {
      variable.logValue(value);
      return;
    }

    for (DTLogTreeNode child : children) {
      child.log(value);
    }
  }

  private void logNull() {
    if (variable != null) {
      variable.logValue(null);
      return;
    }

    for (DTLogTreeNode child : children) {
      child.logNull();
    }
  }

  @Override
  public String toString() {
    if (variable != null) {
      return path;
    }

    StringBuilder builder = new StringBuilder(path);
    builder.append(children[0]);
    for (int i = 1; i < children.length; i++) {
      builder.append('\n')
             .append(children[i]);
    }
    return builder.toString();
  }

  private static boolean isValidName(String name) {
    return name != null && name.length() > 0 && name.indexOf('/') == -1;
  }
}
