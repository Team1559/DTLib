package org.victorrobotics.frc.dtlib.sensor.imu.ctre;

import org.victorrobotics.frc.dtlib.sensor.imu.DTIMU;

import edu.wpi.first.util.sendable.SendableBuilder;

import com.ctre.phoenix.sensors.Pigeon2;

import com.kauailabs.navx.frc.AHRS;

public class DTPigeon2 implements DTIMU<Pigeon2> {
    private final Pigeon2  internal;
    private final double[] angularVelocities;
    private final double[] accelerations;
    private final short[]  rawAccelerations;

    public DTPigeon2(int canID) {
        internal = new Pigeon2(canID);
        angularVelocities = new double[3];
        accelerations = new double[3];
        rawAccelerations = new short[3];
    }

    @Override
    public Pigeon2 internal() {
        return internal;
    }

    @Override
    public void close() throws Exception {
        internal.DestroyObject();
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
    public String getFirmwareVersion() {
        return Integer.toHexString(internal.getFirmwareVersion());
    }

    @Override
    public void zeroYaw() {
        internal.setYaw(0);
    }

    @Override
    public double getCompassHeading() {
        return internal.getAbsoluteCompassHeading();
    }

    @Override
    public double[] getAngularVelocities() {
        internal.getRawGyro(angularVelocities);
        return angularVelocities.clone();
    }

    @Override
    public double[] getAccelerations() {
        internal.getBiasedAccelerometer(rawAccelerations);
        accelerations[0] = parseRawAcceleration(rawAccelerations[0]);
        accelerations[1] = parseRawAcceleration(rawAccelerations[1]);
        accelerations[2] = parseRawAcceleration(rawAccelerations[2]);
        return accelerations.clone();
    }

    private static double parseRawAcceleration(short raw) {
        return raw / 16384D;
    }
}
