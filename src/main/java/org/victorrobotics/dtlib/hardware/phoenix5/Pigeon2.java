package org.victorrobotics.dtlib.hardware.phoenix5;

import org.victorrobotics.dtlib.hardware.IMU;

import com.ctre.phoenix.sensors.Pigeon2_Faults;

public class Pigeon2 implements IMU {
  private final com.ctre.phoenix.sensors.Pigeon2 internal;

  private final short[] rawAccelerations;

  public Pigeon2(int canID) {
    internal = new com.ctre.phoenix.sensors.Pigeon2(canID);
    rawAccelerations = new short[3];
  }

  @Override
  public com.ctre.phoenix.sensors.Pigeon2 getImuImpl() {
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
    double[] angularVelocities = new double[3];
    internal.getRawGyro(angularVelocities);
    return angularVelocities;
  }

  @Override
  public double[] getAccelerations() {
    internal.getBiasedAccelerometer(rawAccelerations);
    double[] accelerations = new double[3];
    accelerations[0] = parseRawAcceleration(rawAccelerations[0]);
    accelerations[1] = parseRawAcceleration(rawAccelerations[1]);
    accelerations[2] = parseRawAcceleration(rawAccelerations[2]);
    return accelerations;
  }

  private static double parseRawAcceleration(short raw) {
    return raw / 16384D;
  }

  public Pigeon2_Faults getFaults() {
    Pigeon2_Faults faults = new Pigeon2_Faults();
    internal.getFaults(faults);
    return faults;
  }
}
