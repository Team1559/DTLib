package org.victorrobotics.dtlib.hardware.phoenix6;

import org.victorrobotics.dtlib.hardware.DTAbsoluteEncoder;
import org.victorrobotics.dtlib.hardware.DTAbsoluteEncoderFaults;

import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class DTCancoder implements DTAbsoluteEncoder {
  private final CANcoder internal;

  private StatusSignal<Double> position;
  private StatusSignal<Double> absolutePosition;
  private StatusSignal<Double> velocity;

  private DTCANCoderFaults faults;
  private String           firmware;

  public DTCancoder(CANcoder cancoder) {
    internal = cancoder;
  }

  public DTCancoder(int canID) {
    this(new CANcoder(canID));
  }

  public DTCancoder(int canID, String canBus) {
    this(new CANcoder(canID, canBus));
  }

  @Override
  public CANcoder getEncoderImpl() {
    return internal;
  }

  @Override
  public Rotation2d getPosition() {
    if (position == null) {
      position = internal.getPosition();
    } else {
      position.refresh();
    }
    return Rotation2d.fromRotations(position.getValue()
                                            .doubleValue());
  }

  @Override
  public Rotation2d getAbsolutePosition() {
    if (absolutePosition == null) {
      absolutePosition = internal.getAbsolutePosition();
    } else {
      absolutePosition.refresh();
    }
    return Rotation2d.fromRotations(absolutePosition.getValue()
                                                    .doubleValue());
  }

  @Override
  public boolean isInverted() {
    MagnetSensorConfigs config = new MagnetSensorConfigs();
    internal.getConfigurator()
            .refresh(config);
    return config.SensorDirection == SensorDirectionValue.CounterClockwise_Positive;
  }

  @Override
  public void setRange(boolean signed) {
    MagnetSensorConfigs config = new MagnetSensorConfigs();
    internal.getConfigurator()
            .refresh(config);
    config.AbsoluteSensorRange = signed ? AbsoluteSensorRangeValue.Signed_PlusMinusHalf
        : AbsoluteSensorRangeValue.Unsigned_0To1;
    internal.getConfigurator()
            .apply(config);
  }

  @Override
  public void setInverted(boolean invert) {
    MagnetSensorConfigs config = new MagnetSensorConfigs();
    internal.getConfigurator()
            .refresh(config);
    config.SensorDirection = invert ? SensorDirectionValue.CounterClockwise_Positive
        : SensorDirectionValue.Clockwise_Positive;
    internal.getConfigurator()
            .apply(config);
  }

  @Override
  public void setPosition(Rotation2d position) {
    internal.setPosition(position.getRotations());
  }

  @Override
  public void setZeroPosition(Rotation2d position) {
    double currentPos = this.position.getValue()
                                     .doubleValue();
    internal.setPosition((currentPos - position.getRotations()) % 1);
  }

  @Override
  public void zeroPosition() {
    internal.setPosition(0);
  }

  @Override
  public void close() {
    internal.close();
  }

  @Override
  public Rotation2d getVelocity() {
    if (velocity == null) {
      velocity = internal.getVelocity();
    } else {
      velocity.refresh();
    }
    return Rotation2d.fromRotations(velocity.getValue()
                                            .doubleValue());
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
  public DTCANCoderFaults getFaults() {
    if (faults == null) {
      faults = new DTCANCoderFaults(internal);
    }
    return faults;
  }

  public static class DTCANCoderFaults implements DTAbsoluteEncoderFaults {
    private final StatusSignal<Integer> allFaults;
    private final StatusSignal<Boolean> badMagnet;
    private final StatusSignal<Boolean> bootDuringEnable;
    private final StatusSignal<Boolean> hardware;
    private final StatusSignal<Boolean> lowVoltage;
    private final StatusSignal<Boolean> unlicensedFeature;

    DTCANCoderFaults(CANcoder internal) {
      allFaults = internal.getFaultField();
      badMagnet = internal.getFault_BadMagnet();
      bootDuringEnable = internal.getFault_BootDuringEnable();
      hardware = internal.getFault_Hardware();
      lowVoltage = internal.getFault_Undervoltage();
      unlicensedFeature = internal.getFault_UnlicensedFeatureInUse();
    }

    @Override
    public boolean lowVoltage() {
      return lowVoltage.getValue()
                       .booleanValue();
    }

    @Override
    public boolean hardwareFailure() {
      return hardware.getValue()
                     .booleanValue()
          || badMagnet.getValue()
                      .booleanValue();
    }

    @Override
    public boolean hasAnyFault() {
      return allFaults.getValue()
                      .intValue() != 0;
    }

    @Override
    public boolean other() {
      return bootDuringEnable.getValue()
                             .booleanValue()
          || unlicensedFeature.getValue()
                              .booleanValue();
    }
  }
}
