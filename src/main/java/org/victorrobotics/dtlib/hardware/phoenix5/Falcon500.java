package org.victorrobotics.dtlib.hardware.phoenix5;

import org.victorrobotics.dtlib.hardware.Motor;

import edu.wpi.first.util.sendable.SendableRegistry;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Falcon500 implements Motor {
  private static final double TICKS_PER_REV      = 2048;
  private static final double SECONDS_PER_MINUTE = 60;
  private static final double MAX_VELOCITY_RPM   = 6380;
  private static final double STALL_TORQUE       = 4.69;

  private final WPI_TalonFX internal;

  private String firmware;

  public Falcon500(int canID) {
    this(canID, "");
  }

  public Falcon500(int canID, String canBus) {
    internal = new WPI_TalonFX(canID, canBus);
    SendableRegistry.remove(internal);
  }

  @Override
  public WPI_TalonFX getMotorImpl() {
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

  public void configFactoryDefault() {
    internal.configFactoryDefault();
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
  public void configPID(int slot, double proportional, double integral, double derivative,
                        double velocityFF, double staticFF, double integralZone) {
    if (slot < 0 || slot > 3) {
      throw new IllegalArgumentException("slot must be in range 0-3");
    }

    // velocityFF unused
    if (Double.isFinite(proportional)) {
      internal.config_kP(slot, proportional);
    }
    if (Double.isFinite(integral)) {
      internal.config_kI(slot, integral);
    }
    if (Double.isFinite(derivative)) {
      internal.config_kD(slot, derivative);
    }
    if (Double.isFinite(staticFF)) {
      internal.config_kF(slot, staticFF);
    }
    if (Double.isFinite(integralZone)) {
      internal.config_IntegralZone(slot, integralZone);
    }
  }

  @Override
  public void configCurrentLimit(int maxSupplyCurrent) {
    internal.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, maxSupplyCurrent,
                                                                          maxSupplyCurrent, 0));
  }

  public void configCurrentLimit(double baseCurrentLimit, double peakCurrentLimit,
                                 double peakDuration) {
    internal.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, baseCurrentLimit,
                                                                          peakCurrentLimit,
                                                                          peakDuration));
  }

  public void configAllSettings(TalonFXConfiguration config) {
    internal.configAllSettings(config);
  }

  @Override
  public boolean isOutputInverted() {
    return internal.getInverted();
  }

  @Override
  public void setPercentOutput(double percent) {
    internal.set(TalonFXControlMode.PercentOutput, percent);
  }

  @Override
  public void setPosition(double position) {
    internal.set(TalonFXControlMode.Position, position * TICKS_PER_REV);
  }

  @Override
  public void setVelocity(double velocity) {
    internal.set(TalonFXControlMode.Velocity, velocity * TICKS_PER_REV / SECONDS_PER_MINUTE * 0.1);
  }

  @Override
  public void neutralOutput() {
    internal.neutralOutput();
  }

  @Override
  public void setEncoderPosition(double position) {
    internal.setSelectedSensorPosition(position * TICKS_PER_REV);
  }

  @Override
  public double getMotorOutputPercent() {
    return internal.getMotorOutputPercent();
  }

  public double getCurrentDraw() {
    return internal.getSupplyCurrent();
  }

  @Override
  public double getTemperature() {
    return internal.getTemperature();
  }

  @Override
  public double getEncoderPosition() {
    return internal.getSelectedSensorPosition() / TICKS_PER_REV;
  }

  @Override
  public double getVelocityRPM() {
    return internal.getSelectedSensorVelocity() / TICKS_PER_REV * 10 * SECONDS_PER_MINUTE;
  }

  public TalonFaults getFaults() {
    Faults faults = new Faults();
    internal.getFaults(faults);
    return new TalonFaults(faults);
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
  public double getInputVoltage() {
    return internal.getBusVoltage();
  }

  @Override
  public double getMaxVelocity() {
    return MAX_VELOCITY_RPM;
  }

  @Override
  public double getStallTorque() {
    return STALL_TORQUE;
  }

  @Override
  public void setPIDSlot(int slot) {
    if (slot < 0 || slot > 3) {
      throw new IllegalArgumentException("slot must be in range 0-3");
    }
    internal.selectProfileSlot(slot, 0);
  }

  @Override
  public double[] getPIDConstants(int slot) {
    if (slot < 0 || slot > 3) {
      throw new IllegalArgumentException("slot must be in range 0-3");
    }
    TalonFXConfiguration allConfigs = new TalonFXConfiguration();
    internal.getAllConfigs(allConfigs);
    SlotConfiguration config = switch (slot) {
      case 1 -> allConfigs.slot1;
      case 2 -> allConfigs.slot2;
      case 3 -> allConfigs.slot3;
      default -> allConfigs.slot0;
    };
    double[] result = new double[6];
    result[0] = config.kP;
    result[1] = config.kI;
    result[2] = config.kD;
    result[3] = Double.NaN; // no velocityFF
    result[4] = config.kF;
    result[5] = config.integralZone;
    return result;
  }
}
