package org.victorrobotics.dtlib.hardware.phoenix5;

import org.victorrobotics.dtlib.hardware.DTMotorFaults;

import com.ctre.phoenix.motorcontrol.Faults;

public class DTTalonFaults implements DTMotorFaults {
  private static final int OTHER_FAULTS_MASK = 0b00111111_10000000;

  private final Faults internal;

  DTTalonFaults(Faults internal) {
    this.internal = internal;
  }

  @Override
  public boolean hasAnyFault() {
    return internal.hasAnyFault();
  }

  @Override
  public boolean lowVoltage() {
    return internal.UnderVoltage;
  }

  @Override
  public boolean other() {
    return (internal.toBitfield() & OTHER_FAULTS_MASK) != 0;
  }

  @Override
  public boolean softLimitForward() {
    return internal.ForwardSoftLimit;
  }

  @Override
  public boolean softLimitReverse() {
    return internal.ReverseSoftLimit;
  }

  @Override
  public boolean hardLimitForward() {
    return internal.ForwardLimitSwitch;
  }

  @Override
  public boolean hardLimitReverse() {
    return internal.ReverseLimitSwitch;
  }

  @Override
  public boolean hasReset() {
    return internal.ResetDuringEn;
  }

  @Override
  public boolean hardwareFailure() {
    return internal.HardwareFailure;
  }
}
