package org.victorrobotics.frc.dtlib.actuator.motor;

public interface DTMotorFaults<FAULT_TYPE> {
    FAULT_TYPE getFaultsImpl();

    boolean hasAnyFault();

    boolean lowVoltage();

    boolean other();

    boolean softLimitForward();

    boolean softLimitReverse();

    boolean hardLimitForward();

    boolean hardLimitReverse();

    boolean hasReset();

    boolean hardwareFailure();
}
