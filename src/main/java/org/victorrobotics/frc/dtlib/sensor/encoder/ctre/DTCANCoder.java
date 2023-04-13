package org.victorrobotics.frc.dtlib.sensor.encoder.ctre;

import org.victorrobotics.frc.dtlib.sensor.encoder.DTAbsoluteEncoder;
import org.victorrobotics.frc.dtlib.sensor.encoder.DTAbsoluteEncoderFaults;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderFaults;

public class DTCANCoder implements DTAbsoluteEncoder {
    private final CANCoder internal;

    private String         firmwareVersion;

    public DTCANCoder(int canID) {
        internal = new CANCoder(canID);

        internal.setPositionToAbsolute();
    }

    public DTCANCoder(int canID, String canBus) {
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
        if (firmwareVersion == null) {
            int version = internal.getFirmwareVersion();
            firmwareVersion = (version >> 8) + "." + (version & 0xFF);
        }
        return firmwareVersion;
    }

    @Override
    public DTAbsoluteEncoderFaults getFaults() {
        CANCoderFaults faults = new CANCoderFaults();
        internal.getFaults(faults);
        return new DTCANCoderFaults(faults);
    }

    public static class DTCANCoderFaults implements DTAbsoluteEncoderFaults {
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
