package org.victorrobotics.frc.dtlib.actuator.motor.ctre;

import org.victorrobotics.frc.dtlib.actuator.motor.DTMotor;
import org.victorrobotics.frc.dtlib.actuator.motor.DTMotorFaults;

import edu.wpi.first.util.sendable.SendableBuilder;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class DTTalonFX implements DTMotor<WPI_TalonFX, SupplyCurrentLimitConfiguration> {
    private static final double TICKS_PER_REV      = 2048;
    private static final double SECONDS_PER_MINUTE = 60;

    private final WPI_TalonFX internal;
    private final int         deviceID;

    private String firmwareVersion;

    private double pidP;
    private double pidI;
    private double pidD;
    private double pidF;
    private double pidIZ;

    private boolean brakeEnabled;

    public DTTalonFX(int canID) {
        this(canID, "");
    }

    public DTTalonFX(int canID, String canBus) {
        internal = new WPI_TalonFX(canID, canBus);
        this.deviceID = canID;
    }

    @Override
    public WPI_TalonFX internal() {
        return internal;
    }

    /* Write config params */

    @Override
    public void configBrakeMode(boolean enable) {
        internal.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
        brakeEnabled = enable;
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
    public void configPIDproportional(double coefficient) {
        internal.config_kP(0, coefficient);
        pidP = coefficient;
    }

    @Override
    public void configPIDintegral(double coefficient) {
        internal.config_kI(0, coefficient);
        pidI = coefficient;
    }

    @Override
    public void configPIDderivative(double coefficient) {
        internal.config_kD(0, coefficient);
        pidD = coefficient;
    }

    @Override
    public void configPIDfeedforward(double coefficient) {
        internal.config_kF(0, coefficient);
        pidF = coefficient;
    }

    @Override
    public void configPIDintegralZone(double iZone) {
        internal.config_IntegralZone(0, iZone);
        pidIZ = iZone;
    }

    @Override
    public void configCurrentLimit(SupplyCurrentLimitConfiguration maxCurrent) {
        internal.configSupplyCurrentLimit(maxCurrent);
    }

    public void configAllSettings(TalonFXConfiguration config) {
        internal.configAllSettings(config);
    }

    /* Read config params */

    @Override
    public boolean isOutputInverted() {
        return internal.getInverted();
    }

    @Override
    public boolean isBrakeEnabled() {
        return brakeEnabled;
    }

    @Override
    public double getPIDproportional() {
        return pidP;
    }

    @Override
    public double getPIDintegral() {
        return pidI;
    }

    @Override
    public double getPIDderivative() {
        return pidD;
    }

    @Override
    public double getPIDfeedforward() {
        return pidF;
    }

    @Override
    public double getPIDintegralZone() {
        return pidIZ;
    }

    /* Set motor status */

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
        internal.set(TalonFXControlMode.Velocity,
                velocity * TICKS_PER_REV / SECONDS_PER_MINUTE * 0.1);
    }

    @Override
    public void neutralOutput() {
        internal.neutralOutput();
    }

    @Override
    public void setEncoderPosition(double position) {
        internal.setSelectedSensorPosition(position * TICKS_PER_REV);
    }

    /* Read motor status */

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
    public double getEncoderVelocity() {
        return internal.getSelectedSensorVelocity() / TICKS_PER_REV * 10;
    }

    public DTTalonFXFaults getFaults() {
        Faults faults = new Faults();
        internal.getFaults(faults);
        return new DTTalonFXFaults(faults);
    }

    @Override
    public String getFirmwareVersion() {
        if (firmwareVersion == null) {
            int version = internal.getFirmwareVersion();
            firmwareVersion = (version >> 8) + "." + (version & 0xFF);
        }
        return firmwareVersion;
    }

    @Override
    public void close() {
        internal.close();
    }

    @Override
    public void customSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Current", this::getCurrentDraw, null);
    }

    @Override
    public int getCanID() {
        return deviceID;
    }

    @Override
    public double getInputVoltage() {
        return internal.getBusVoltage();
    }

    public static class DTTalonFXFaults implements DTMotorFaults<Faults> {
        private static final int OTHER_FAULTS_MASK = 0b0;

        private final Faults internal;

        DTTalonFXFaults(Faults internal) {
            this.internal = internal;
        }

        @Override
        public Faults internal() {
            return internal;
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
}
