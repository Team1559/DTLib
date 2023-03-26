package org.victorrobotics.frc.dtlib.network;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public interface DTSendable extends Sendable {
    double UPDATE_RATE_FAST_HZ = 25;
    double UPDATE_RATE_STD_HZ  = 10;
    double UPDATE_RATE_SLOW_HZ = 2;

    @Override
    void initSendable(SendableBuilder builder);
}
