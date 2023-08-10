package org.victorrobotics.frc.dtlib.log;

import org.victorrobotics.frc.dtlib.DTRobot;

import java.lang.reflect.AnnotatedElement;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

public class DTDataLogSearcher {
    public static List<Field> getImmediateLoggableFields(Class<? extends DTRobot> clazz) {
        List<Field> fields = new ArrayList<>();
        while (clazz != DTRobot.class) {
            for (Field f : clazz.getDeclaredFields()) {
                if (shouldLog(f)) {
                    fields.add(f);
                }
            }
            clazz = clazz.getSuperclass()
                         .asSubclass(DTRobot.class);
        }
        fields.addAll(List.of(DTRobot.class.getDeclaredFields()));
        fields.sort(DTDataLogSearcher::compareFields);

        return fields;
    }

    private static boolean shouldLog(Field f) {
        return hasLogAnnotation(f) && (isLoggable(f.getType()) || isPrimitive(f.getType()));
    }

    private static boolean hasLogAnnotation(AnnotatedElement e) {
        return e.isAnnotationPresent(DTDataLog.class);
    }

    private static boolean isLoggable(Class<?> clazz) {
        return false;
    }

    private static boolean isPrimitive(Class<?> clazz) {
        return clazz.isPrimitive();
    }

    private static boolean isVoid(Class<?> clazz) {
        return clazz == Void.class;
    }

    private static int compareFields(Field f1, Field f2) {
        int classCompare = compareClasses(f1.getDeclaringClass(), f2.getDeclaringClass());
        if (classCompare != 0) {
            return classCompare;
        }

        int modCompare = compareModifiers(f1.getModifiers(), f2.getModifiers());
        if (modCompare != 0) {
            return modCompare;
        }

        return f1.getName()
                 .compareTo(f2.getName());
    }

    private static <T> int compareClasses(Class<? extends T> clazz1, Class<? extends T> clazz2) {
        // Check if they belong to different classes, determine superclass
        boolean super1 = false;
        boolean super2 = false;
        try {
            clazz2.asSubclass(clazz1);
            super1 = true;
        } catch (ClassCastException e) {
            // dummy
        }
        try {
            clazz1.asSubclass(clazz2);
            super2 = true;
        } catch (ClassCastException e) {
            // dummy
        }
        // Superclass first
        if (super1 && !super2) {
            return -1;
        } else if (super2 && !super1) {
            return 1;
        }
        // Same class, or relationship cannot be determined
        return 0;
    }

    public static void init(DTRobot robot) {
        Map<Class<? extends DTRobot>, List<Field>> fields = new HashMap<>();
        Map<Class<? extends DTRobot>, List<Method>> methods = new HashMap<>();
        Class<? extends DTRobot> clazz = robot.getClass();
        while (clazz != DTRobot.class) {
            fields.put(clazz, new ArrayList<>(List.of(clazz.getDeclaredFields())));
            methods.put(clazz, new ArrayList<>(List.of(clazz.getDeclaredMethods())));
            clazz = clazz.getSuperclass()
                         .asSubclass(DTRobot.class);
        }
        fields.put(clazz, new ArrayList<>(List.of(clazz.getDeclaredFields())));
        methods.put(clazz, new ArrayList<>(List.of(clazz.getDeclaredMethods())));
        for (Entry<Class<? extends DTRobot>, List<Field>> entry : fields.entrySet()) {
            entry.getValue()
                 .removeIf(f -> !f.isAnnotationPresent(DTDataLog.class) || !f.getType()
                                                                             .isPrimitive());
            entry.getValue()
                 .sort((Field f1, Field f2) -> {
                     int c = compareModifiers(f1.getModifiers(), f2.getModifiers());
                     return c != 0 ? c
                             : f1.getName()
                                 .compareTo(f2.getName());
                 });
        }
        for (Entry<Class<? extends DTRobot>, List<Method>> entry : methods.entrySet()) {
            entry.getValue()
                 .removeIf(m -> !m.isAnnotationPresent(DTDataLog.class) || m.getParameterTypes().length != 0
                         || Void.class.isAssignableFrom(m.getReturnType()));
            entry.getValue()
                 .sort((Method m1, Method m2) -> m1.getName()
                                                   .compareTo(m2.getName()));
        }
        fields.entrySet()
              .removeIf(e -> e.getValue()
                              .isEmpty());
        methods.entrySet()
               .removeIf(e -> e.getValue()
                               .isEmpty());
    }

    private static String toString(Field f) {
        return (Modifier.toString(f.getModifiers()) + " " + f.getType()
                                                             .getSimpleName()
                + " " + f.getName() + ";").trim();
    }

    private static String toString(Method m) {
        return (Modifier.toString(m.getModifiers()) + " " + m.getReturnType()
                                                             .getSimpleName()
                + " " + m.getName() + List.of(m.getParameterTypes())
                                          .toString()
                                          .replace('[', '(')
                                          .replace(']', ')')).trim();
    }

    private static int compareModifiers(int m1, int m2) {
        int xor = m1 ^ m2;
        if (xor == 0) {
            // all same
            return 0;
        }

        if (Modifier.isStatic(xor)) {
            return Modifier.isStatic(m1) ? -1 : 1;
        } else if (Modifier.isFinal(xor)) {
            return Modifier.isFinal(m1) ? -1 : 1;
        }

        if (Modifier.isPublic(xor)) {
            return Modifier.isPublic(m1) ? -1 : 1;
        } else if (Modifier.isProtected(xor)) {
            return Modifier.isProtected(m1) ? -1 : 1;
        } else if (Modifier.isPrivate(xor)) {
            return Modifier.isPrivate(m1) ? 1 : -1;
        }

        return m2 - m1;
    }
}
