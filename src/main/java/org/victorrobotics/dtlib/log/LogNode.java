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
import java.util.function.UnaryOperator;

public class LogNode {
  private final String                path;
  private final Class<?>              type;
  private final UnaryOperator<Object> getter;

  private LogNode[] children;
  private LogVariable        variable;

  public LogNode(String path, String name, Class<?> type, UnaryOperator<Object> getter) {
    this.path = path + "/" + name;
    this.type = type;
    this.getter = getter;
  }

  protected final void init(Deque<Class<?>> stack, Set<Class<?>> clazzes,
      List<StaticLogVariable> staticVars) {
    if (stack.contains(type)) {
      // Prevent infinite recursion
      return;
    }

    List<LogNode> childList = new ArrayList<>();
    Class<?> clazz = type;
    do {
      if (childList.isEmpty()) {
        LogType logType = LogWriter.LOG_TYPES.get(clazz);
        if (logType != null) {
          variable = new LogVariable(logType, path);
          return;
        }
      }

      boolean includeStatic = clazzes.add(clazz);

      for (Field field : clazz.getDeclaredFields()) {
        initField(field, childList, staticVars, includeStatic);
      }

      for (Method method : clazz.getDeclaredMethods()) {
        initMethod(method, childList, staticVars, includeStatic);
      }

      clazz = clazz.getSuperclass();
    } while (clazz != null);

    stack.addFirst(type);
    initChildren(stack, clazzes, staticVars, childList);
    stack.removeFirst();

    children = childList.isEmpty() ? null : childList.toArray(LogNode[]::new);
  }

  private void initField(Field field, List<LogNode> childList,
      List<StaticLogVariable> staticVars, boolean includeStatic) {
    DTLog annotation = field.getAnnotation(DTLog.class);
    if (annotation == null) {
      return;
    }

    boolean isStatic = Modifier.isStatic(field.getModifiers());
    if (isStatic && !includeStatic) {
      return;
    }

    if (!field.trySetAccessible()) {
      return;
    }

    String name = annotation.value();
    if (!isValidName(name)) {
      name = field.getName();
    }

    if (isStatic) {
      LogType logType = LogWriter.LOG_TYPES.get(field.getType());
      if (logType == null) {
        return;
      }

      staticVars.add(new StaticLogVariable(logType, field.getDeclaringClass(), name, () -> {
        try {
          return field.get(null);
        } catch (IllegalAccessException | IllegalArgumentException e) {
          return null;
        }
      }));
      return;
    }

    childList.add(new LogNode(path, name, field.getType(), parent -> {
      try {
        return field.get(parent);
      } catch (IllegalAccessException | IllegalArgumentException e) {
        return null;
      }
    }));
  }

  private void initMethod(Method method, List<LogNode> childList,
      List<StaticLogVariable> staticVars, boolean includeStatic) {
    DTLog annotation = method.getAnnotation(DTLog.class);
    if (annotation == null) {
      return;
    }

    if (method.getParameterCount() != 0 || method.getReturnType() == void.class) {
      return;
    }

    boolean isStatic = Modifier.isStatic(method.getModifiers());
    if (isStatic && !includeStatic) {
      return;
    }

    if (!method.trySetAccessible()) {
      return;
    }

    String name = annotation.value();
    if (!isValidName(name)) {
      name = method.getName() + "()";
    }

    if (isStatic) {
      LogType logType = LogWriter.LOG_TYPES.get(method.getReturnType());
      if (logType == null) {
        return;
      }

      staticVars.add(new StaticLogVariable(logType, method.getDeclaringClass(), name, () -> {
        try {
          return method.invoke(null, (Object[]) null);
        } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
          return null;
        }
      }));
      return;
    }

    childList.add(new LogNode(path, name, method.getReturnType(), parent -> {
      try {
        return method.invoke(parent, (Object[]) null);
      } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
        return null;
      }
    }));
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

    for (LogNode child : children) {
      child.log(value);
    }
  }

  @SuppressWarnings("unchecked")
  private void logNull() {
    if (variable != null) {
      variable.logValue(null);
      return;
    }

    for (LogNode child : children) {
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

  private static void initChildren(Deque<Class<?>> stack, Set<Class<?>> clazzes,
      List<StaticLogVariable> staticVars, List<LogNode> childList) {
    Iterator<LogNode> itr = childList.iterator();
    while (itr.hasNext()) {
      LogNode child = itr.next();
      child.init(stack, clazzes, staticVars);
      if (child.variable == null && child.children == null) {
        itr.remove();
      }
    }
  }

  private static boolean isValidName(String name) {
    return name != null && name.length() > 0 && name.indexOf('/') == -1;
  }
}
