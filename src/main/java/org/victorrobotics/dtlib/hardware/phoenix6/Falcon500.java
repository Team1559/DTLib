package org.victorrobotics.dtlib.hardware.phoenix6;

import org.victorrobotics.dtlib.hardware.Motor;
import org.victorrobotics.dtlib.hardware.MotorFaults;

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

public class Falcon500 implements Motor {
  private static final double MAX_VELOCITY_RPM = 6380;
  private static final double STALL_TORQUE     = 4.69;

  private final TalonFX internal;

  private StatusSignal<Double> position;
  private StatusSignal<Double> velocity;
  private StatusSignal<Double> supplyCurrent;
  private StatusSignal<Double> voltage;
  private StatusSignal<Double> temperature;

  private DTTalonFXFaults faults;
  private String          firmware;
  private int             pidSlot;

  public Falcon500(TalonFX motor) {
    internal = motor;
  }

  public Falcon500(int canID) {
    this(new TalonFX(canID));
  }

  public Falcon500(int canID, String canBus) {
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
  public void configPID(int slot, double proportional, double integral, double derivative,
                        double velocityFF, double staticFF, double integralZone) {
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
      throw new IllegalArgumentException("slot must be in range 0-2");
    }
  }

  @Override
  public void configCurrentLimit(int maxSupplyCurrent) {
    configCurrentLimit(maxSupplyCurrent, maxSupplyCurrent, 0);
  }

  public void configCurrentLimit(double baseCurrentLimit, double peakCurrentLimit,
                                 double peakDuration) {
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
      throw new IllegalArgumentException("slot must be in range 0-2");
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
    internal.setControl(new PositionDutyCycle(position, 0, false, 0, pidSlot, false));
  }

  @Override
  public void setVelocity(double velocity) {
    internal.setControl(new VelocityDutyCycle(velocity, 0, false, 0, pidSlot, false));
  }

  @Override
  public void neutralOutput() {
    internal.setControl(new NeutralOut());
  }

  @Override
  public void setEncoderPosition(double position) {
    internal.setPosition(position, 0);
  }

  @Override
  public double getMotorOutputPercent() {
    throw new UnsupportedOperationException("Unimplemented method 'getMotorOutputPercent'");
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

  public static class DTTalonFXFaults implements MotorFaults {
    private final StatusSignal<Integer> allFaults;
    private final StatusSignal<Boolean> bootDuringEnable;
    private final StatusSignal<Boolean> deviceTemp;
    private final StatusSignal<Boolean> forwardHardLimit;
    private final StatusSignal<Boolean> forwardSoftLimit;
    private final StatusSignal<Boolean> fusedSensorOutOfSync;
    private final StatusSignal<Boolean> hardware;
    private final StatusSignal<Boolean> overSupplyV;
    private final StatusSignal<Boolean> procTemp;
    private final StatusSignal<Boolean> reverseHardLimit;
    private final StatusSignal<Boolean> reverseSoftLimit;
    private final StatusSignal<Boolean> statorCurrLimit;
    private final StatusSignal<Boolean> supplyCurrLimit;
    private final StatusSignal<Boolean> undervoltage;
    private final StatusSignal<Boolean> unlicensedFeatureInUse;
    private final StatusSignal<Boolean> unstableSupplyV;
    private final StatusSignal<Boolean> usingFusedCANcoderWhileUnlicensed;
    private final StatusSignal<Boolean> bridgeBrownout;

    DTTalonFXFaults(TalonFX internal) {
      allFaults = internal.getFaultField();
      bootDuringEnable = internal.getFault_BootDuringEnable();
      deviceTemp = internal.getFault_DeviceTemp();
      forwardHardLimit = internal.getFault_ForwardHardLimit();
      forwardSoftLimit = internal.getFault_ForwardSoftLimit();
      fusedSensorOutOfSync = internal.getFault_FusedSensorOutOfSync();
      hardware = internal.getFault_Hardware();
      overSupplyV = internal.getFault_OverSupplyV();
      procTemp = internal.getFault_ProcTemp();
      reverseHardLimit = internal.getFault_ReverseHardLimit();
      reverseSoftLimit = internal.getFault_ReverseSoftLimit();
      statorCurrLimit = internal.getFault_StatorCurrLimit();
      supplyCurrLimit = internal.getFault_SupplyCurrLimit();
      undervoltage = internal.getFault_Undervoltage();
      unlicensedFeatureInUse = internal.getFault_UnlicensedFeatureInUse();
      unstableSupplyV = internal.getFault_UnstableSupplyV();
      usingFusedCANcoderWhileUnlicensed = internal.getFault_UsingFusedCANcoderWhileUnlicensed();
      bridgeBrownout = internal.getFault_BridgeBrownout();
    }

    @Override
    public boolean hasAnyFault() {
      return allFaults.getValue()
                      .intValue()
          != 0;
    }

    @Override
    public boolean lowVoltage() {
      return undervoltage.getValue()
                         .booleanValue()
          || bridgeBrownout.getValue()
                           .booleanValue();
    }

    @Override
    public boolean other() {
      return overSupplyV.getValue()
                        .booleanValue()
          || statorCurrLimit.getValue()
                            .booleanValue()
          || supplyCurrLimit.getValue()
                            .booleanValue()
          || unlicensedFeatureInUse.getValue()
                                   .booleanValue()
          || unstableSupplyV.getValue()
                            .booleanValue()
          || usingFusedCANcoderWhileUnlicensed.getValue()
                                              .booleanValue();
    }

    @Override
    public boolean softLimitForward() {
      return forwardSoftLimit.getValue()
                             .booleanValue();
    }

    @Override
    public boolean softLimitReverse() {
      return reverseSoftLimit.getValue()
                             .booleanValue();
    }

    @Override
    public boolean hardLimitForward() {
      return forwardHardLimit.getValue()
                             .booleanValue();
    }

    @Override
    public boolean hardLimitReverse() {
      return reverseHardLimit.getValue()
                             .booleanValue();
    }

    @Override
    public boolean hasReset() {
      return bootDuringEnable.getValue()
                             .booleanValue();
    }

    @Override
    public boolean hardwareFailure() {
      return hardware.getValue()
                     .booleanValue()
          || deviceTemp.getValue()
                       .booleanValue()
          || procTemp.getValue()
                     .booleanValue()
          || fusedSensorOutOfSync.getValue()
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
