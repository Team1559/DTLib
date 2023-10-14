package org.victorrobotics.dtlib.hardware.phoenix6;

import org.victorrobotics.dtlib.hardware.IMU;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

public class DTPigeon2 implements IMU {
  private final Pigeon2 internal;

  private StatusSignal<Double> yaw;
  private StatusSignal<Double> pitch;
  private StatusSignal<Double> roll;

  private StatusSignal<Double> angularVelocityX;
  private StatusSignal<Double> angularVelocityY;
  private StatusSignal<Double> angularVelocityZ;

  private StatusSignal<Double> accelerationX;
  private StatusSignal<Double> accelerationY;
  private StatusSignal<Double> accelerationZ;

  private StatusSignal<Integer> faults;
  private String                firmware;

  public DTPigeon2(int canID) {
    this(new Pigeon2(canID));
  }

  public DTPigeon2(int canID, String canBus) {
    this(new Pigeon2(canID, canBus));
  }

  public DTPigeon2(Pigeon2 pigeon) {
    internal = pigeon;
  }

  @Override
  public Pigeon2 getImuImpl() {
    return internal;
  }

  @Override
  public void close() {
    internal.close();
  }

  @Override
  public double getYaw() {
    if (yaw == null) {
      yaw = internal.getYaw();
    } else {
      yaw.refresh();
    }
    return yaw.getValue()
              .doubleValue();
  }

  @Override
  public double getPitch() {
    if (pitch == null) {
      pitch = internal.getPitch();
    } else {
      pitch.refresh();
    }
    return pitch.getValue()
                .doubleValue();
  }

  @Override
  public double getRoll() {
    if (roll == null) {
      roll = internal.getRoll();
    } else {
      roll.refresh();
    }
    return roll.getValue()
               .doubleValue();
  }

  @Override
  public String getFirmwareVersion() {
    if (firmware == null) {
      int v = internal.getVersion()
                      .getValue()
                      .intValue();
      firmware = new StringBuilder().append((v >> 24) & 0xFF)
                                    .append('.')
                                    .append((v >> 16) & 0xFF)
                                    .append('.')
                                    .append((v >> 8) & 0xFF)
                                    .append('.')
                                    .append(v & 0xFF)
                                    .toString();
    }
    return firmware;
  }

  @Override
  public void zeroYaw() {
    internal.setYaw(0, 0.005);
  }

  @Override
  public double getCompassHeading() {
    // TODO: revise API or solve implementation
    throw new UnsupportedOperationException("Unimplemented method 'getCompassHeading'");
  }

  @Override
  public double[] getAngularVelocities() {
    if (angularVelocityX == null || angularVelocityY == null || angularVelocityZ == null) {
      angularVelocityX = internal.getAngularVelocityX();
      angularVelocityY = internal.getAngularVelocityY();
      angularVelocityZ = internal.getAngularVelocityZ();
    } else {
      angularVelocityX.refresh();
      angularVelocityY.refresh();
      angularVelocityZ.refresh();
    }

    double[] result = new double[3];
    result[0] = angularVelocityX.getValue()
                                .doubleValue();
    result[1] = angularVelocityY.getValue()
                                .doubleValue();
    result[2] = angularVelocityZ.getValue()
                                .doubleValue();
    return result;
  }

  @Override
  public double[] getAccelerations() {
    if (accelerationX == null || accelerationY == null || accelerationZ == null) {
      accelerationX = internal.getAccelerationX();
      accelerationY = internal.getAccelerationY();
      accelerationZ = internal.getAccelerationZ();
    } else {
      accelerationX.refresh();
      accelerationY.refresh();
      accelerationZ.refresh();
    }

    double[] result = new double[3];
    result[0] = accelerationX.getValue()
                             .doubleValue();
    result[1] = accelerationY.getValue()
                             .doubleValue();
    result[2] = accelerationZ.getValue()
                             .doubleValue();
    return result;
  }

  public int getFaults() {
    if (faults == null) {
      faults = internal.getFaultField();
    } else {
      faults.refresh();
    }

    return faults.getValue()
                 .intValue();
  }
}
