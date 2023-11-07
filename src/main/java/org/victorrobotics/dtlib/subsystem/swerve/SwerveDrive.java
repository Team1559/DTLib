package org.victorrobotics.dtlib.subsystem.swerve;

import org.victorrobotics.dtlib.math.geometry.Vector2D;
import org.victorrobotics.dtlib.math.geometry.Vector2D_R;
import org.victorrobotics.dtlib.math.kinematics.SwerveDriveKinematics;
import org.victorrobotics.dtlib.math.kinematics.SwerveDriveOdometry;
import org.victorrobotics.dtlib.math.kinematics.SwerveModulePosition;
import org.victorrobotics.dtlib.math.kinematics.SwerveModuleState;
import org.victorrobotics.dtlib.math.trajectory.AccelerationLimit;
import org.victorrobotics.dtlib.math.trajectory.VelocityLimit;
import org.victorrobotics.dtlib.subsystem.Subsystem;

import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public abstract class SwerveDrive extends Subsystem {
  private final SwerveModule[]        modules;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry   poseEstimator;

  private final SwerveModulePosition[] positions;

  private Vector2D          centerOfRotation;
  private AccelerationLimit accelerationLimit;
  private VelocityLimit     velocityLimit;
  private boolean           isFieldRelative;

  private Field2d    virtualField;
  private Vector2D_R currentSpeeds;

  protected SwerveDrive(SwerveModule... modules) {
    if (modules == null || modules.length < 2) {
      throw new IllegalArgumentException("Swerve drive requires at least 2 wheels");
    }

    this.modules = new SwerveModule[modules.length];
    for (int i = 0; i < modules.length; i++) {
      this.modules[i] = Objects.requireNonNull(modules[i]);
    }

    Vector2D[] wheelLocations = new Vector2D[modules.length];
    positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < wheelLocations.length; i++) {
      wheelLocations[i] = modules[i].getLocation();
      positions[i] = modules[i].getPosition();
    }

    kinematics = new SwerveDriveKinematics(5, wheelLocations);
    poseEstimator = new SwerveDriveOdometry(kinematics, new Pose2d(), getGyroAngle(), positions,
                                            new double[] { 0.1, 0.1, 0.1 });

    centerOfRotation = new Vector2D();
    accelerationLimit = new AccelerationLimit();
    currentSpeeds = new Vector2D_R();
  }

  public void initializeHardware() {}

  public final void setCenterOfRotation(Vector2D newCenterOfRotation) {
    centerOfRotation = newCenterOfRotation;
  }

  public void setAccelerationLimit(AccelerationLimit limit) {
    accelerationLimit = limit;
  }

  public void setVelocityLimit(VelocityLimit limit) {
    velocityLimit = limit;
  }

  public void setFieldRelative() {
    isFieldRelative = true;
  }

  public void setRobotRelative() {
    isFieldRelative = false;
  }

  protected abstract Rotation2d getGyroAngle();

  public final void driveVelocity(double vx, double vy, double vr) {
    driveVelocity(vx, vy, vr, centerOfRotation);
  }

  public void driveVelocity(double vx, double vy, double vr, Vector2D centerOfRotation) {
    Vector2D_R previousSpeeds = currentSpeeds;

    if (isFieldRelative) {
      ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vr, getGyroAngle());
      currentSpeeds = new Vector2D_R(speeds);
    } else {
      currentSpeeds.setX(vx);
      currentSpeeds.setY(vy);
      currentSpeeds.setR(vr);
    }

    velocityLimit.apply(currentSpeeds);
    accelerationLimit.apply(currentSpeeds, previousSpeeds);

    kinematics.setCenterOfRotation(centerOfRotation);
    SwerveModuleState[] newStates = kinematics.computeModuleStates(currentSpeeds);
    setStates(newStates);
  }

  public final void setStates(SwerveModuleState... states) {
    if (states.length != modules.length) {
      throw new IllegalArgumentException("received " + states.length + " module states for "
          + modules.length + " modules");
    }

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
    }
  }

  public void holdPosition() {
    for (SwerveModule module : modules) {
      // Orient wheels towards center of robot so they all collide
      module.holdPosition(Rotation2d.fromRadians(module.getLocation()
                                                       .theta())
                                    .unaryMinus());
    }
  }

  @Override
  public void close() {}
}
