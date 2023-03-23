package org.victorrobotics.frc.dtlib.drivetrain.swerve;

import com.deviltech.frc.dtlib.HardwareComponent;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule extends HardwareComponent {
    void setState(SwerveModuleState state);

    void holdPosition(Rotation2d steerAngle);

    SwerveModuleState getState();

    SwerveModulePosition getPosition();

    Translation2d getLocation();
}
