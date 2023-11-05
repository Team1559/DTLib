package org.victorrobotics.dtlib.subsystem.swerve;

import org.victorrobotics.dtlib.math.geometry.Vector2D;
import org.victorrobotics.dtlib.math.kinematics.SwerveModulePosition;
import org.victorrobotics.dtlib.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModule {
  Vector2D getLocation();

  void setState(SwerveModuleState state);

  void holdPosition(Rotation2d steerAngle);

  Rotation2d getSteerAngle();

  double getVelocity();

  default SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), getSteerAngle());
  }

  SwerveModulePosition getPosition();
}
