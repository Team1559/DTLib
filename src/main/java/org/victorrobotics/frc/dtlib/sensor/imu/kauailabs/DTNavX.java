package org.victorrobotics.frc.dtlib.sensor.imu.kauailabs;

import org.victorrobotics.frc.dtlib.sensor.imu.DTIMU;

import edu.wpi.first.util.sendable.SendableRegistry;

import com.kauailabs.navx.frc.AHRS;

public class DTNavX implements DTIMU {
    private final AHRS internal;

    public DTNavX() {
        internal = new AHRS();
        SendableRegistry.remove(internal);
    }

    @Override
    public void close() {
        try {
            internal.close();
        } catch (Exception e) {
            // ignore
        }
    }

    @Override
    public AHRS getImuImpl() {
        return internal;
    }

    @Override
    public double getYaw() {
        return internal.getYaw();
    }

    @Override
    public double getPitch() {
        return internal.getPitch();
    }

    @Override
    public double getRoll() {

        return internal.getRoll();
    }

    @Override
    public void zeroYaw() {
        internal.zeroYaw();
    }

    @Override
    public String getFirmwareVersion() {
        return internal.getFirmwareVersion();
    }

    @Override
    public double getCompassHeading() {
        return internal.getCompassHeading();
    }

    @Override
    public double[] getAngularVelocities() {
        return new double[] { internal.getRawGyroX(), internal.getRawGyroY(), internal.getRawGyroZ() };
    }

    @Override
    public double[] getAccelerations() {
        return new double[] { internal.getWorldLinearAccelX(), internal.getWorldLinearAccelY(),
                internal.getWorldLinearAccelZ() };
    }

}
