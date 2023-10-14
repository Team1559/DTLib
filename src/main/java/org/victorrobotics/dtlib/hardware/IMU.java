package org.victorrobotics.dtlib.hardware;

public interface IMU {
  Object getImuImpl();

  double getYaw();

  double getPitch();

  double getRoll();

  void zeroYaw();

  String getFirmwareVersion();

  double getCompassHeading();

  double[] getAngularVelocities();

  double[] getAccelerations();
}
