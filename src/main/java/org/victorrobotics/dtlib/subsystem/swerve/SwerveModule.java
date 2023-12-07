package org.victorrobotics.dtlib.subsystem.swerve;

import org.victorrobotics.dtlib.math.geometry.Vector2D;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModule {
  Rotation2d getAngle();

  void setAngle(Rotation2d angle);

  double getSpeed();

  void setSpeed(double speed);

  double getDistance();

  Vector2D getLocation();

  double maxSpeed();
}
