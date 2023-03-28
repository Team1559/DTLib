package org.victorrobotics.frc.dtlib;

import org.victorrobotics.frc.dtlib.command.test.DTTestCommand;
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
    private final String       identifier;

    private final Set<DTHardwareComponent> components;

    protected DTSubsystem(String typeName, String name) {
        setSubsystem(typeName);
        setName(name);
        identifier = typeName + "-" + name;
        dashboardTable = DTDash.getMainTable()
                               .getSubTable(identifier);
        components = new HashSet<>();
    }

    public NetworkTable getDashboardTable() {
        return dashboardTable;
    }

    public String getIdentifier() {
        return identifier;
    }

    protected final void addComponent(DTHardwareComponent component) {
        if (components.contains(component)) {
            // Already registered here
            return;
        } else if (!ALL_COMPONENTS.add(component)) {
            // Component has already been added to another subsystem
            throw new DTIllegalArgumentException(
                    "components should not belong to multiple subsystems", component);
        }

        components.add(component);
    }

    public abstract DTTestCommand getSelfTestCommand();
}
