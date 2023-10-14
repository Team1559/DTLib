package org.victorrobotics.dtlib.hardware.phoenix5;

import org.victorrobotics.dtlib.hardware.AbsoluteEncoder;
import org.victorrobotics.dtlib.hardware.AbsoluteEncoderFaults;

import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderFaults;

public class DTCancoder implements AbsoluteEncoder {
  private final CANCoder internal;

  private String firmware;

  public DTCancoder(int canID) {
    this(canID, "");
  }

  public DTCancoder(int canID, String canBus) {
    internal = new CANCoder(canID, canBus);
  }

  @Override
  public CANCoder getEncoderImpl() {
    return internal;
  }

  public void configFactoryDefault() {
    internal.configFactoryDefault();
  }

  @Override
  public Rotation2d getPosition() {
    return Rotation2d.fromDegrees(internal.getPosition());
  }

  @Override
  public Rotation2d getAbsolutePosition() {
    return Rotation2d.fromDegrees(internal.getAbsolutePosition());
  }

  public boolean isInverted() {
    return internal.configGetSensorDirection();
  }

  @Override
  public void setRange(boolean signed) {
    internal.configAbsoluteSensorRange(signed ? AbsoluteSensorRange.Signed_PlusMinus180
        : AbsoluteSensorRange.Unsigned_0_to_360);
  }

  @Override
  public void setInverted(boolean invert) {
    internal.configSensorDirection(invert);
  }

  @Override
  public void setPosition(Rotation2d position) {
    internal.setPosition(position.getDegrees() % 360);
  }

  @Override
  public void setZeroPosition(Rotation2d position) {
    double currentPos = internal.getAbsolutePosition();
    internal.setPosition((currentPos - position.getDegrees()) % 360);
  }

  @Override
  public void zeroPosition() {
    internal.setPosition(0);
  }

  @Override
  public void close() {
    // nothing here
  }

  @Override
  public Rotation2d getVelocity() {
    return Rotation2d.fromDegrees(internal.getVelocity());
  }

  @Override
  public String getFirmwareVersion() {
    if (firmware == null) {
      int v = internal.getFirmwareVersion();
      firmware = new StringBuilder().append((v >> 8) & 0xFF)
                                    .append('.')
                                    .append(v & 0xFF)
                                    .toString();
    }
    return firmware;
  }

  @Override
  public AbsoluteEncoderFaults getFaults() {
    CANCoderFaults faults = new CANCoderFaults();
    internal.getFaults(faults);
    return new DTCANCoderFaults(faults);
  }

  public static class DTCANCoderFaults implements AbsoluteEncoderFaults {
    private final CANCoderFaults internal;

    DTCANCoderFaults(CANCoderFaults internal) {
      this.internal = internal;
    }

    @Override
    public boolean lowVoltage() {
      return internal.UnderVoltage;
    }

    @Override
    public boolean hardwareFailure() {
      return internal.HardwareFault || internal.MagnetTooWeak;
    }

    @Override
    public boolean hasAnyFault() {
      return internal.hasAnyFault();
    }

    @Override
    public boolean other() {
      return internal.APIError || internal.ResetDuringEn;
    }
  }
}
