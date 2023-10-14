package org.victorrobotics.dtlib.subsystem.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
  Translation2d getLocation();

  void setState(SwerveModuleState state);

  void holdPosition(Rotation2d steerAngle);

  Rotation2d getSteerAngle();

  double getVelocity();

  default SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), getSteerAngle());
  }

  SwerveModulePosition getPosition();
}
