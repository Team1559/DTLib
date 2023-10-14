package org.victorrobotics.dtlib.hardware.revrobotics;

import org.victorrobotics.dtlib.exception.DTIllegalArgumentException;
import org.victorrobotics.dtlib.hardware.Motor;
import org.victorrobotics.dtlib.hardware.MotorFaults;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class Neo implements Motor {
  private static final double MAX_VELOCITY_RPM = 5676;
  private static final double STALL_TORQUE     = 2.6;

  private final CANSparkMax           internal;
  private final SparkMaxPIDController pidController;
  private final RelativeEncoder       encoder;

  private int pidSlot;

  public Neo(int canID) {
    internal = new CANSparkMax(canID, MotorType.kBrushless);
    pidController = internal.getPIDController();
    encoder = internal.getEncoder();
  }

  @Override
  public void close() {
    internal.close();
  }

  @Override
  public CANSparkMax getMotorImpl() {
    return internal;
  }

  @Override
  public void configBrakeMode(boolean enable) {
    internal.setIdleMode(enable ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
  }

  @Override
  public void configOutputInverted(boolean invert) {
    internal.setInverted(invert);
  }

  @Override
  public void configOpenLoopRampRate(double seconds0To100) {
    internal.setOpenLoopRampRate(seconds0To100);
  }

  @Override
  public void configClosedLoopRampRate(double seconds0To100) {
    internal.setClosedLoopRampRate(seconds0To100);
  }

  @Override
  public void configCurrentLimit(int maxCurrentDraw) {
    internal.setSmartCurrentLimit(maxCurrentDraw, maxCurrentDraw);
  }

  @Override
  public boolean isOutputInverted() {
    return internal.getInverted();
  }

  @Override
  public void configPID(int slot, double proportional, double integral, double derivative,
                        double velocityFF, double staticFF, double integralZone) {
    if (slot < 0 || slot > 3) {
      throw new DTIllegalArgumentException(slot, "slot must be in range 0-3");
    }

    // No velocityFF
    if (Double.isFinite(proportional)) {
      pidController.setP(proportional, slot);
    }
    if (Double.isFinite(integral)) {
      pidController.setI(integral, slot);
    }
    if (Double.isFinite(derivative)) {
      pidController.setD(derivative, slot);
    }
    if (Double.isFinite(staticFF)) {
      pidController.setFF(staticFF, slot);
    }
    if (Double.isFinite(integralZone)) {
      pidController.setIZone(integralZone, slot);
    }
  }

  @Override
  public void setPIDSlot(int slot) {
    if (slot < 0 || slot > 3) {
      throw new DTIllegalArgumentException(slot, "slot must be in range 0-3");
    }
    pidSlot = slot;
  }

  @Override
  public double[] getPIDConstants(int slot) {
    double[] result = new double[6];
    result[0] = pidController.getP(slot);
    result[1] = pidController.getI(slot);
    result[2] = pidController.getD(slot);
    result[3] = Double.NaN;
    result[4] = pidController.getFF(slot);
    result[5] = pidController.getIZone(slot);
    return result;
  }

  @Override
  public void setPercentOutput(double percent) {
    internal.set(percent);
  }

  @Override
  public void setPosition(double position) {
    pidController.setReference(position, ControlType.kPosition, pidSlot);
  }

  @Override
  public void setVelocity(double velocity) {
    pidController.setReference(velocity, ControlType.kVelocity, pidSlot);
  }

  @Override
  public void neutralOutput() {
    internal.stopMotor();
  }

  @Override
  public void setEncoderPosition(double position) {
    encoder.setPosition(position);
  }

  @Override
  public double getMotorOutputPercent() {
    return internal.getAppliedOutput();
  }

  @Override
  public double getInputVoltage() {
    return internal.getBusVoltage();
  }

  @Override
  public double getTemperature() {
    return internal.getMotorTemperature();
  }

  @Override
  public double getEncoderPosition() {
    return encoder.getPosition();
  }

  @Override
  public double getVelocityRPM() {
    return encoder.getVelocity();
  }

  @Override
  public DTNeoFaults getFaults() {
    return new DTNeoFaults(internal.getFaults());
  }

  @Override
  public String getFirmwareVersion() {
    return internal.getFirmwareString();
  }

  public static class DTNeoFaults implements MotorFaults {
    private static final short OTHER_FAULTS_MASK = 0b00001101_11110110;

    private final short internal;

    DTNeoFaults(short internal) {
      this.internal = internal;
    }

    @Override
    public boolean hasAnyFault() {
      return internal != 0;
    }

    @Override
    public boolean lowVoltage() {
      return (internal & (1 << CANSparkMax.FaultID.kBrownout.value)) != 0;
    }

    @Override
    public boolean other() {
      return (internal & OTHER_FAULTS_MASK) != 0;
    }

    @Override
    public boolean softLimitForward() {
      return (internal & (1 << CANSparkMax.FaultID.kSoftLimitFwd.value)) != 0;
    }

    @Override
    public boolean softLimitReverse() {
      return (internal & (1 << CANSparkMax.FaultID.kSoftLimitRev.value)) != 0;
    }

    @Override
    public boolean hardLimitForward() {
      return (internal & (1 << CANSparkMax.FaultID.kHardLimitFwd.value)) != 0;
    }

    @Override
    public boolean hardLimitReverse() {
      return (internal & (1 << CANSparkMax.FaultID.kHardLimitRev.value)) != 0;
    }

    @Override
    public boolean hasReset() {
      return (internal & (1 << CANSparkMax.FaultID.kHasReset.value)) != 0;
    }

    @Override
    public boolean hardwareFailure() {
      return (internal & (1 << CANSparkMax.FaultID.kMotorFault.value)) != 0;
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
