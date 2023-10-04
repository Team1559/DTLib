package org.victorrobotics.dtlib.drivetrain.swerve;

import org.victorrobotics.dtlib.DTHardwareComponent;
import org.victorrobotics.dtlib.DTSubsystem;
import org.victorrobotics.dtlib.drivetrain.DTAccelerationLimit;
import org.victorrobotics.dtlib.drivetrain.DTVelocityLimit;
import org.victorrobotics.dtlib.exception.DTIllegalArgumentException;
import org.victorrobotics.dtlib.math.geometry.DTVector2d;
import org.victorrobotics.dtlib.math.geometry.DTVector2dR;

import java.util.Objects;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public abstract class DTSwerveDrive extends DTSubsystem implements DTHardwareComponent {
  private final DTSwerveModule[]         modules;
  private final SwerveDriveKinematics    kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;

  private final SwerveModulePosition[] positions;

  private DTVector2d          centerOfRotation;
  private DTAccelerationLimit accelerationLimit;
  private DTVelocityLimit     velocityLimit;
  private boolean             isFieldRelative;

  private Field2d     virtualField;
  private DTVector2dR currentSpeeds;

  protected DTSwerveDrive(DTSwerveModule... modules) {
    Objects.requireNonNull(modules);
    if (modules.length < 2) {
      throw new IllegalArgumentException("Swerve drive requires at least 2 wheels");
    }
    for (DTSwerveModule m : modules) {
      Objects.requireNonNull(m);
    }
    this.modules = modules.clone();

    setSubsystem("Swerve Drive");
    setName(getSubsystem());

    initializeHardware();

    Translation2d[] wheelLocations = new Translation2d[modules.length];
    positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < wheelLocations.length; i++) {
      wheelLocations[i] = modules[i].getLocation();
      positions[i] = modules[i].getPosition();
    }

    kinematics = new SwerveDriveKinematics(wheelLocations);
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroAngle(), positions,
        new Pose2d());

    centerOfRotation = new DTVector2d();
    accelerationLimit = new DTAccelerationLimit();
    currentSpeeds = new DTVector2dR();
  }

  public void initializeHardware() {
    for (DTSwerveModule module : modules) {
      module.initializeHardware();
    }
  }

  public final void setCenterOfRotation(DTVector2d newCenterOfRotation) {
    centerOfRotation = newCenterOfRotation;
  }

  public void setAccelerationLimit(DTAccelerationLimit limit) {
    accelerationLimit = limit;
  }

  public void setVelocityLimit(DTVelocityLimit limit) {
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

  public void driveVelocity(double vx, double vy, double vr, DTVector2d centerOfRotation) {
    DTVector2dR previousSpeeds = currentSpeeds;

    if (isFieldRelative) {
      ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vr, getGyroAngle());
      currentSpeeds = new DTVector2dR(speeds);
    } else {
      currentSpeeds.setX(vx);
      currentSpeeds.setY(vy);
      currentSpeeds.setR(vr);
    }

    velocityLimit.apply(currentSpeeds);
    accelerationLimit.apply(currentSpeeds, previousSpeeds);

    SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(currentSpeeds.toChassisSpeeds(),
        centerOfRotation.toTranslation2d());
    setStates(newStates);
  }

  public final void setStates(SwerveModuleState... states) {
    if (states.length != modules.length) {
      throw new DTIllegalArgumentException(states,
          "received " + states.length + " module states for " + modules.length + " modules");
    }

    double minCosine = 1;
    for (int i = 0; i < modules.length; i++) {
      states[i] = SwerveModuleState.optimize(states[i], modules[i].getSteerAngle());
      double cosine = modules[i].getSteerAngle()
                                .minus(states[i].angle)
                                .getCos();
      if (cosine < minCosine) {
        minCosine = cosine;
      }
    }
    minCosine *= minCosine * minCosine; // minCosine to the 3rd power
    for (int i = 0; i < modules.length; i++) {
      states[i].speedMetersPerSecond *= minCosine;
      modules[i].setState(states[i]);
    }
  }

  public void holdPosition() {
    for (DTSwerveModule module : modules) {
      // Orient wheels towards center of robot so they all collide
      module.holdPosition(module.getLocation()
                                .getAngle()
                                .unaryMinus());
    }
  }

  @Override
  public void close() {
    for (DTSwerveModule module : modules) {
      module.close();
    }
  }
}
