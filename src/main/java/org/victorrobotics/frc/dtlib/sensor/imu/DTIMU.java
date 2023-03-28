package org.victorrobotics.frc.dtlib.sensor.imu;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public interface DTIMU<IMUTYPE> extends Sendable, AutoCloseable {
    IMUTYPE getImuImpl();

    double getYaw();

    double getPitch();

    double getRoll();

    void zeroYaw();

    @Override
    default void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Roll", this::getRoll, null);
        builder.addDoubleProperty("Pitch", this::getPitch, null);
        builder.addDoubleProperty("Yaw", this::getYaw, null);
        builder.addDoubleArrayProperty("Angular velocity", this::getAngularVelocities, null);
        builder.addDoubleProperty("Compass", this::getCompassHeading, null);
        builder.addStringProperty("Firmware", this::getFirmwareVersion, null);

        customizeSendable(builder);
    }

    default void customizeSendable(SendableBuilder builder) {}

    String getFirmwareVersion();

    double getCompassHeading();

    double[] getAngularVelocities();

    double[] getAccelerations();
}
