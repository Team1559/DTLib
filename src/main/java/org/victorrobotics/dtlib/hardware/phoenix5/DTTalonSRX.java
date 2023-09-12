package org.victorrobotics.dtlib.hardware.phoenix5;

import org.victorrobotics.dtlib.exception.DTIllegalArgumentException;
import org.victorrobotics.dtlib.hardware.DTMotor;
import org.victorrobotics.dtlib.hardware.DTMotorFaults;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class DTTalonSRX implements DTMotor {
  private final WPI_TalonSRX internal;

  private String firmware;

  public DTTalonSRX(WPI_TalonSRX motor) {
    internal = motor;
  }

  public DTTalonSRX(int canID) {
    this(new WPI_TalonSRX(canID));
  }

  @Override
  public void close() {
    internal.close();
  }

  @Override
  public WPI_TalonSRX getMotorImpl() {
    return internal;
  }

  @Override
  public void configBrakeMode(boolean enable) {
    internal.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
  }

  @Override
  public void configOutputInverted(boolean invert) {
    internal.setInverted(invert);
  }

  @Override
  public void configOpenLoopRampRate(double seconds0To100) {
    internal.configOpenloopRamp(seconds0To100);
  }

  @Override
  public void configClosedLoopRampRate(double seconds0To100) {
    internal.configClosedloopRamp(seconds0To100);
  }

  @Override
  public void configPID(int slot, double proportional, double integral, double derivative, double velocityFF,
      double staticFF, double integralZone) {
    if (slot < 0 || slot > 3) {
      throw new DTIllegalArgumentException(slot, "slot must be in range 0-3");
    }

    // No velocityFF
    if (!Double.isNaN(proportional)) {
      internal.config_kP(slot, proportional);
    }
    if (!Double.isNaN(integral)) {
      internal.config_kI(slot, integral);
    }
    if (!Double.isNaN(derivative)) {
      internal.config_kD(slot, derivative);
    }
    if (!Double.isNaN(staticFF)) {
      internal.config_kF(slot, staticFF);
    }
    if (!Double.isNaN(integralZone)) {
      internal.config_IntegralZone(slot, integralZone);
    }
  }

  @Override
  public void setPIDSlot(int slot) {
    internal.selectProfileSlot(slot, 0);
  }

  @Override
  public double[] getPIDConstants(int slot) {
    // Can't read constants from TalonSRX; cache?
    return new double[6];
  }

  @Override
  public void configCurrentLimit(int maxSupplyCurrent) {
    internal.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, maxSupplyCurrent, maxSupplyCurrent, 0));
  }

  @Override
  public boolean isOutputInverted() {
    return internal.getInverted();
  }

  @Override
  public void setPercentOutput(double percent) {
    internal.set(TalonSRXControlMode.PercentOutput, percent);
  }

  @Override
  public void setPosition(double position) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
  }

  @Override
  public void setVelocity(double velocity) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVelocity'");
  }

  @Override
  public void neutralOutput() {
    internal.neutralOutput();
  }

  @Override
  public void setEncoderPosition(double position) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setEncoderPosition'");
  }

  @Override
  public double getMotorOutputPercent() {
    return internal.getMotorOutputPercent();
  }

  @Override
  public double getInputVoltage() {
    return internal.getBusVoltage();
  }

  @Override
  public double getTemperature() {
    return internal.getTemperature();
  }

  @Override
  public double getEncoderPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getEncoderPosition'");
  }

  @Override
  public double getVelocityRPM() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getVelocityRPM'");
  }

  @Override
  public DTMotorFaults getFaults() {
    Faults faults = new Faults();
    internal.getFaults(faults);
    return new DTTalonFaults(faults);
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
  public double getMaxVelocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getMaxVelocity'");
  }

  @Override
  public double getStallTorque() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getStallTorque'");
  }
}
