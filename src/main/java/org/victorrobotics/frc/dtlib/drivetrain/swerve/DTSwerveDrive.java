package org.victorrobotics.frc.dtlib.drivetrain.swerve;

import java.util.Objects;

import org.victorrobotics.frc.dtlib.DTHardwareComponent;
import org.victorrobotics.frc.dtlib.drivetrain.DTAccelerationLimit;
import org.victorrobotics.frc.dtlib.drivetrain.DTVelocityLimit;
import org.victorrobotics.frc.dtlib.exception.DTIllegalArgumentException;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DTSwerveDrive extends SubsystemBase implements DTHardwareComponent {
    private final DTSwerveModule[]         modules;
    private final SwerveDriveKinematics    kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final SwerveModulePosition[] positions;

    private Translation2d       centerOfRotation;
    private DTAccelerationLimit accelerationLimit;
    private DTVelocityLimit     velocityLimit;
    private boolean             isFieldRelative;

    private Field2d       virtualField;
    private ChassisSpeeds previousSpeeds;
    private ChassisSpeeds currentSpeeds;

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

        centerOfRotation = new Translation2d();
        accelerationLimit = new DTAccelerationLimit();
        currentSpeeds = new ChassisSpeeds();
    }

    public void initializeHardware() {
        for (DTSwerveModule module : modules) {
            module.initializeHardware();
        }
    }

    public final void setCenterOfRotation(Translation2d newCenterOfRotation) {
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

    public void driveVelocity(double vx, double vy, double vr, Translation2d centerOfRotation) {
        previousSpeeds = currentSpeeds;

        if (isFieldRelative) {
            currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vr, getGyroAngle());
        } else {
            currentSpeeds = new ChassisSpeeds(vx, vy, vr);
        }

        velocityLimit.apply(currentSpeeds);
        accelerationLimit.apply(currentSpeeds, previousSpeeds);

        SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(currentSpeeds,
                centerOfRotation);
        setStates(newStates);
    }

    public final void setStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new DTIllegalArgumentException("received " + states.length + " module states for "
                    + modules.length + " modules", (Object[]) states);
        }

        double minCosine = 1;
        for (int i = 0; i < modules.length; i++) {
            states[i] = SwerveModuleState.optimize(states[i], modules[i].getSteerAngle());
            double cosine = Math.cos(Units.degreesToRadians(modules[i].getSteerAngle()
                                                                      .getDegrees()
                    - states[i].angle.getDegrees()));
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
            module.holdPosition(module.getLocation().getAngle());
        }
    }

    @Override
    public void close() {
        for (DTSwerveModule module : modules) {
            module.close();
        }
    }
}
