package org.victorrobotics.frc.dtlib.actuator.motor.ctre;

import org.victorrobotics.frc.dtlib.actuator.motor.DTMotor;
import org.victorrobotics.frc.dtlib.actuator.motor.DTMotorFaults;
import org.victorrobotics.frc.dtlib.exception.DTIllegalArgumentException;

import edu.wpi.first.util.sendable.SendableBuilder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.ejml.simple.UnsupportedOperation;

public class DTTalonFX_v6 implements DTMotor {
  private static final double MAX_VELOCITY_RPM = 6380;
  private static final double STALL_TORQUE     = 4.69;

  private final TalonFX internal;

  private StatusSignal<Double> position;
  private StatusSignal<Double> velocity;
  private StatusSignal<Double> supplyCurrent;
  private StatusSignal<Double> voltage;
  private StatusSignal<Double> temperature;

  private DTTalonFXFaults faults;
  private String          firmwareVersion;
  private int             pidSlot;

  public DTTalonFX_v6(TalonFX motor) {
    internal = motor;
  }

  public DTTalonFX_v6(int canID) {
    this(new TalonFX(canID));
  }

  public DTTalonFX_v6(int canID, String canBus) {
    this(new TalonFX(canID, canBus));
  }

  @Override
  public TalonFX getMotorImpl() {
    return internal;
  }

  @Override
  public void configBrakeMode(boolean enable) {
    MotorOutputConfigs config = new MotorOutputConfigs();
    internal.getConfigurator()
            .refresh(config);
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    internal.getConfigurator()
            .apply(config);
  }

  @Override
  public void configOutputInverted(boolean invert) {
    internal.setInverted(invert);
  }

  public void configFactoryDefault() {
    internal.getConfigurator()
            .apply(new TalonFXConfiguration());
  }

  @Override
  public void configOpenLoopRampRate(double seconds0To100) {
    OpenLoopRampsConfigs configs = new OpenLoopRampsConfigs();
    internal.getConfigurator()
            .refresh(configs);
    configs.DutyCycleOpenLoopRampPeriod = seconds0To100;
    configs.TorqueOpenLoopRampPeriod = seconds0To100;
    configs.VoltageOpenLoopRampPeriod = seconds0To100;
    internal.getConfigurator()
            .apply(configs);

  }

  @Override
  public void configClosedLoopRampRate(double seconds0To100) {
    ClosedLoopRampsConfigs configs = new ClosedLoopRampsConfigs();
    internal.getConfigurator()
            .refresh(configs);
    configs.DutyCycleClosedLoopRampPeriod = seconds0To100;
    configs.TorqueClosedLoopRampPeriod = seconds0To100;
    configs.VoltageClosedLoopRampPeriod = seconds0To100;
    internal.getConfigurator()
            .apply(configs);
  }

  @Override
  public void configPID(int slot, double proportional, double integral, double derivative, double velocityFF,
      double staticFF, double integralZone) {
    // integralZone is deprecated in v6, windup prevented automatically
    if (slot == 0) {
      Slot0Configs configs = new Slot0Configs();
      internal.getConfigurator()
              .refresh(configs);
      if (Double.isFinite(proportional)) {
        configs.kP = proportional;
      }
      if (Double.isFinite(integral)) {
        configs.kI = integral;
      }
      if (Double.isFinite(derivative)) {
        configs.kD = derivative;
      }
      if (Double.isFinite(velocityFF)) {
        configs.kV = velocityFF;
      }
      if (Double.isFinite(staticFF)) {
        configs.kS = staticFF;
      }
      internal.getConfigurator()
              .apply(configs);
    } else if (slot == 1) {
      Slot1Configs configs = new Slot1Configs();
      internal.getConfigurator()
              .refresh(configs);
      if (Double.isFinite(proportional)) {
        configs.kP = proportional;
      }
      if (Double.isFinite(integral)) {
        configs.kI = integral;
      }
      if (Double.isFinite(derivative)) {
        configs.kD = derivative;
      }
      if (Double.isFinite(velocityFF)) {
        configs.kV = velocityFF;
      }
      if (Double.isFinite(staticFF)) {
        configs.kS = staticFF;
      }
      internal.getConfigurator()
              .apply(configs);
    } else if (slot == 2) {
      Slot2Configs configs = new Slot2Configs();
      internal.getConfigurator()
              .refresh(configs);
      if (Double.isFinite(proportional)) {
        configs.kP = proportional;
      }
      if (Double.isFinite(integral)) {
        configs.kI = integral;
      }
      if (Double.isFinite(derivative)) {
        configs.kD = derivative;
      }
      if (Double.isFinite(velocityFF)) {
        configs.kV = velocityFF;
      }
      if (Double.isFinite(staticFF)) {
        configs.kS = staticFF;
      }
      internal.getConfigurator()
              .apply(configs);
    } else {
      throw new DTIllegalArgumentException(slot, "slot must be in range 0-2");
    }
  }

  @Override
  public void configCurrentLimit(int maxSupplyCurrent) {
    configCurrentLimit(maxSupplyCurrent, maxSupplyCurrent, 0);
  }

  public void configCurrentLimit(double baseCurrentLimit, double peakCurrentLimit, double peakDuration) {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
    internal.getConfigurator()
            .refresh(configs);
    configs.SupplyCurrentLimitEnable = true;
    configs.SupplyCurrentLimit = baseCurrentLimit;
    configs.SupplyCurrentThreshold = peakCurrentLimit;
    configs.SupplyTimeThreshold = peakDuration;
    internal.getConfigurator()
            .apply(configs);
  }

  @Override
  public boolean isOutputInverted() {
    return internal.getInverted();
  }

  @Override
  public double[] getPIDConstants(int slot) {
    double[] result = new double[6];
    result[5] = Double.NaN; // No IZ
    if (slot == 0) {
      Slot0Configs configs = new Slot0Configs();
      internal.getConfigurator()
              .refresh(configs);
      result[0] = configs.kP;
      result[1] = configs.kI;
      result[2] = configs.kD;
      result[3] = configs.kV;
      result[4] = configs.kS;
    } else if (slot == 1) {
      Slot1Configs configs = new Slot1Configs();
      internal.getConfigurator()
              .refresh(configs);
      result[0] = configs.kP;
      result[1] = configs.kI;
      result[2] = configs.kD;
      result[3] = configs.kV;
      result[4] = configs.kS;
    } else if (slot == 2) {
      Slot2Configs configs = new Slot2Configs();
      internal.getConfigurator()
              .refresh(configs);
      result[0] = configs.kP;
      result[1] = configs.kI;
      result[2] = configs.kD;
      result[3] = configs.kV;
      result[4] = configs.kS;
    } else {
      throw new DTIllegalArgumentException(slot, "slot must be in range 0-2");
    }
    return result;
  }

  @Override
  public void setPIDSlot(int slot) {
    pidSlot = slot;
  }

  @Override
  public void setPercentOutput(double percent) {
    internal.setControl(new DutyCycleOut(percent));
  }

  @Override
  public void setPosition(double position) {
    internal.setControl(new PositionDutyCycle(position, false, 0, 0, false));
  }

  @Override
  public void setVelocity(double velocity) {
    internal.setControl(new VelocityDutyCycle(velocity, false, 0, pidSlot, false));
  }

  @Override
  public void neutralOutput() {
    internal.setControl(new NeutralOut());
  }

  @Override
  public void setEncoderPosition(double position) {
    internal.setRotorPosition(position);
  }

  @Override
  public double getMotorOutputPercent() {
    throw new UnsupportedOperation("Unimplemented method 'getMotorOutputPercent'");
  }

  public double getCurrentDraw() {
    if (supplyCurrent == null) {
      supplyCurrent = internal.getSupplyCurrent();
    } else {
      supplyCurrent.refresh();
    }
    return supplyCurrent.getValue()
                        .doubleValue();
  }

  @Override
  public double getTemperature() {
    if (temperature == null) {
      temperature = internal.getDeviceTemp();
    } else {
      temperature.refresh();
    }
    return temperature.getValue()
                      .doubleValue();
  }

  @Override
  public double getEncoderPosition() {
    if (position == null) {
      position = internal.getPosition();
    } else {
      position.refresh();
    }
    return position.getValue()
                   .doubleValue();
  }

  @Override
  public double getVelocityRPM() {
    if (velocity == null) {
      velocity = internal.getVelocity();
    } else {
      velocity.refresh();
    }
    return velocity.getValue()
                   .doubleValue();
  }

  public DTTalonFXFaults getFaults() {
    if (faults == null) {
      faults = new DTTalonFXFaults(internal);
    }
    return faults;
  }

  @Override
  public String getFirmwareVersion() {
    if (firmwareVersion == null) {
      int version = internal.getVersion()
                            .getValue()
                            .intValue();
      StringBuilder builder = new StringBuilder();
      builder.append(version >> 24);
      builder.append('.');
      builder.append((version >> 16) & 0xFF);
      builder.append('.');
      builder.append((version >> 8) & 0xFF);
      builder.append('.');
      builder.append(version & 0xFF);
      firmwareVersion = builder.toString();
    }
    return firmwareVersion;
  }

  @Override
  public void close() {
    internal.close();
  }

  @Override
  public void customizeSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Current", this::getCurrentDraw, null);
  }

  @Override
  public double getInputVoltage() {
    if (voltage == null) {
      voltage = internal.getSupplyVoltage();
    } else {
      voltage.refresh();
    }
    return voltage.getValue()
                  .doubleValue();
  }

  public static class DTTalonFXFaults implements DTMotorFaults {
    private final StatusSignal<Integer> allFaults;
    private final StatusSignal<Boolean> BootDuringEnable;
    private final StatusSignal<Boolean> DeviceTemp;
    private final StatusSignal<Boolean> ForwardHardLimit;
    private final StatusSignal<Boolean> ForwardSoftLimit;
    private final StatusSignal<Boolean> FusedSensorOutOfSync;
    private final StatusSignal<Boolean> Hardware;
    private final StatusSignal<Boolean> MissingRemoteSensor;
    private final StatusSignal<Boolean> OverSupplyV;
    private final StatusSignal<Boolean> ProcTemp;
    private final StatusSignal<Boolean> ReverseHardLimit;
    private final StatusSignal<Boolean> ReverseSoftLimit;
    private final StatusSignal<Boolean> StatorCurrLimit;
    private final StatusSignal<Boolean> SupplyCurrLimit;
    private final StatusSignal<Boolean> Undervoltage;
    private final StatusSignal<Boolean> UnlicensedFeatureInUse;
    private final StatusSignal<Boolean> UnstableSupplyV;
    private final StatusSignal<Boolean> UsingFusedCANcoderWhileUnlicensed;

    DTTalonFXFaults(TalonFX internal) {
      allFaults = internal.getFaultField();
      BootDuringEnable = internal.getFault_BootDuringEnable();
      DeviceTemp = internal.getFault_DeviceTemp();
      ForwardHardLimit = internal.getFault_ForwardHardLimit();
      ForwardSoftLimit = internal.getFault_ForwardSoftLimit();
      FusedSensorOutOfSync = internal.getFault_FusedSensorOutOfSync();
      Hardware = internal.getFault_Hardware();
      MissingRemoteSensor = internal.getFault_MissingRemoteSensor();
      OverSupplyV = internal.getFault_OverSupplyV();
      ProcTemp = internal.getFault_ProcTemp();
      ReverseHardLimit = internal.getFault_ReverseHardLimit();
      ReverseSoftLimit = internal.getFault_ReverseSoftLimit();
      StatorCurrLimit = internal.getFault_StatorCurrLimit();
      SupplyCurrLimit = internal.getFault_SupplyCurrLimit();
      Undervoltage = internal.getFault_Undervoltage();
      UnlicensedFeatureInUse = internal.getFault_UnlicensedFeatureInUse();
      UnstableSupplyV = internal.getFault_UnstableSupplyV();
      UsingFusedCANcoderWhileUnlicensed = internal.getFault_UsingFusedCANcoderWhileUnlicensed();
    }

    @Override
    public boolean hasAnyFault() {
      return allFaults.getValue()
                      .intValue() != 0;
    }

    @Override
    public boolean lowVoltage() {
      return Undervoltage.getValue()
                         .booleanValue();
    }

    @Override
    public boolean other() {
      return OverSupplyV.getValue()
                        .booleanValue()
          || StatorCurrLimit.getValue()
                            .booleanValue()
          || SupplyCurrLimit.getValue()
                            .booleanValue()
          || UnlicensedFeatureInUse.getValue()
                                   .booleanValue()
          || UnstableSupplyV.getValue()
                            .booleanValue()
          || UsingFusedCANcoderWhileUnlicensed.getValue()
                                              .booleanValue();
    }

    @Override
    public boolean softLimitForward() {
      return ForwardSoftLimit.getValue()
                             .booleanValue();
    }

    @Override
    public boolean softLimitReverse() {
      return ReverseSoftLimit.getValue()
                             .booleanValue();
    }

    @Override
    public boolean hardLimitForward() {
      return ForwardHardLimit.getValue()
                             .booleanValue();
    }

    @Override
    public boolean hardLimitReverse() {
      return ReverseHardLimit.getValue()
                             .booleanValue();
    }

    @Override
    public boolean hasReset() {
      return BootDuringEnable.getValue()
                             .booleanValue();
    }

    @Override
    public boolean hardwareFailure() {
      return Hardware.getValue()
                     .booleanValue()
          || DeviceTemp.getValue()
                       .booleanValue()
          || ProcTemp.getValue()
                     .booleanValue()
          || FusedSensorOutOfSync.getValue()
                                 .booleanValue()
          || MissingRemoteSensor.getValue()
                                .booleanValue();
    }
  }

  @Override
  public double getMaxVelocity() {
    return MAX_VELOCITY_RPM;
  }

  @Override
  public double getStallTorque() {
    return STALL_TORQUE;
  }
}
