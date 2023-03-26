package org.victorrobotics.frc.dtlib.logging;

import java.util.HashMap;
import java.util.Map;

public class DTLogFormat {
    private final Map<String, Class<?>> variables;

    public DTLogFormat() {
        variables = new HashMap<>();
    }
}
