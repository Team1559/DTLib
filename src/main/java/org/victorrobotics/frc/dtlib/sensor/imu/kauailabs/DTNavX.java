package org.victorrobotics.frc.dtlib.sensor.imu.kauailabs;

import org.victorrobotics.frc.dtlib.sensor.imu.DTIMU;

import edu.wpi.first.util.sendable.SendableRegistry;

import com.kauailabs.navx.frc.AHRS;

public class DTNavX implements DTIMU<AHRS> {
    private final AHRS internal;

    public DTNavX() {
        internal = new AHRS();
        SendableRegistry.remove(internal);
    }

    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'close'");
    }

    @Override
    public AHRS internal() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'internal'");
    }

    @Override
    public double getYaw() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getYaw'");
    }

    @Override
    public double getPitch() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPitch'");
    }

    @Override
    public double getRoll() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRoll'");
    }

    @Override
    public void zeroYaw() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'zeroYaw'");
    }

    @Override
    public String getFirmwareVersion() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getFirmwareVersion'");
    }

    @Override
    public double getCompassHeading() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getCompassHeading'");
    }

    @Override
    public double[] getAngularVelocities() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAngularVelocities'");
    }

    @Override
    public double[] getAccelerations() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAccelerations'");
    }

}
