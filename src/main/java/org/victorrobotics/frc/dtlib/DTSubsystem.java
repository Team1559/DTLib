package org.victorrobotics.frc.dtlib;

import org.victorrobotics.frc.dtlib.dashboard.DTDash;
import org.victorrobotics.frc.dtlib.exception.DTIllegalArgumentException;
import org.victorrobotics.frc.dtlib.network.DTSendable;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DTSubsystem extends SubsystemBase implements DTSendable {
    private static final Set<DTHardwareComponent> ALL_COMPONENTS = new HashSet<>();

    private final NetworkTable dashboardTable;
    private final String       name;

    private final Set<DTHardwareComponent> components;

    protected DTSubsystem(String name) {
        this.name = name;
        dashboardTable = DTDash.getMainTable()
                               .getSubTable(name);
        components = new HashSet<>();
    }

    protected final void addComponent(DTHardwareComponent component) {
        if (components.contains(component)) {
            // Already registered here
            return;
        }

        if (!ALL_COMPONENTS.add(component)) {
            // Component has already been added to another subsystem
            throw new DTIllegalArgumentException(
                    "components should not belong to multiple subsystems", component);
        }

        components.add(component);
    }
}
