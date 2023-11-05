package org.victorrobotics.dtlib.hardware.kauailabs;

import org.victorrobotics.dtlib.hardware.IMU;

import edu.wpi.first.util.sendable.SendableRegistry;

import com.kauailabs.navx.frc.AHRS;

public class NavX implements IMU {
  private final AHRS internal;

  public NavX() {
    internal = new AHRS();
    SendableRegistry.remove(internal);
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
