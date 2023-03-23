package org.victorrobotics.frc.dtlib.drivetrain.swerve;

import java.util.Objects;

import org.victorrobotics.frc.dtlib.DTHardwareComponent;
import org.victorrobotics.frc.dtlib.drivetrain.DTAccelerationLimit;
import org.victorrobotics.frc.dtlib.drivetrain.DTVelocityLimit;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DTSwerveDrive extends SubsystemBase implements DTHardwareComponent {
    private final DTSwerveModule[]         modules;
    private final SwerveDriveKinematics    kinematics;
    private SwerveDrivePoseEstimator    poseEstimator;

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

        Translation2d[] wheelLocations = new Translation2d[modules.length];
        for (int i = 0; i < wheelLocations.length; i++) {
            wheelLocations[i] = modules[i].getLocation();
        }
        kinematics = new SwerveDriveKinematics(wheelLocations);

        centerOfRotation = new Translation2d();
        accelerationLimit = new DTAccelerationLimit();
        currentSpeeds = new ChassisSpeeds();
    }

    public void initializeHardware() {
        for (DTSwerveModule module : modules) {
            module.initializeHardware();
        }
    }

    public void setCenterOfRotation(Translation2d newCenterOfRotation) {
        centerOfRotation = newCenterOfRotation;
    }

    public void setAccelerationLimit(DTAccelerationLimit limit) {
        accelerationLimit = limit;
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

        kinematics.toSwerveModuleStates(currentSpeeds, centerOfRotation);
    }
}
