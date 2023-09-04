package org.victorrobotics.frc.dtlib.actuator.motor.revrobotics;

import org.victorrobotics.frc.dtlib.actuator.motor.DTMotor;
import org.victorrobotics.frc.dtlib.actuator.motor.DTMotorFaults;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class DTNeo implements DTMotor {
  private static final double MAX_VELOCITY_RPM = 5676;
  private static final double STALL_TORQUE     = 2.6;

  private final CANSparkMax           internal;
  private final SparkMaxPIDController pidController;
  private final RelativeEncoder       encoder;
  private final int                   deviceID;

  public DTNeo(int canID) {
    deviceID = canID;
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
  public int getCanID() {
    return deviceID;
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
  public void configPIDproportional(double proportional) {
    pidController.setP(proportional);
  }

  @Override
  public void configPIDintegral(double integral) {
    pidController.setI(integral);
  }

  @Override
  public void configPIDderivative(double derivative) {
    pidController.setD(derivative);
  }

  @Override
  public void configPIDfeedforward(double feedforward) {
    pidController.setFF(feedforward);
  }

  @Override
  public void configPIDintegralZone(double iZone) {
    pidController.setIZone(iZone);
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
  public boolean isBrakeEnabled() {
    return internal.getIdleMode() == CANSparkMax.IdleMode.kBrake;
  }

  @Override
  public double getPIDproportional() {
    return pidController.getP();
  }

  @Override
  public double getPIDintegral() {
    return pidController.getI();
  }

  @Override
  public double getPIDderivative() {
    return pidController.getD();
  }

  @Override
  public double getPIDfeedforward() {
    return pidController.getFF();
  }

  @Override
  public double getPIDintegralZone() {
    return pidController.getIZone();
  }

  @Override
  public void setPercentOutput(double percent) {
    internal.set(percent);
  }

  @Override
  public void setPosition(double position) {
    encoder.setPosition(position);
  }

  @Override
  public void setVelocity(double velocity) {
    pidController.setReference(velocity, ControlType.kVelocity);
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

  public static class DTNeoFaults implements DTMotorFaults {
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
