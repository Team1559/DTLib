package org.victorrobotics.frc.dtlib.drivetrain.swerve;

import java.util.Objects;

import com.deviltech.frc.dtlib.HardwareComponent;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public abstract class SwerveDrive implements HardwareComponent {
    private SwerveModule[]        modules;
    private SwerveDriveKinematics kinematics;
    private Translation2d         centerOfRotation;
    private boolean               isFieldRelative;
    private double                maximumLinearVelocity;
    private double                maximumAngularVelocity;

    protected SwerveDrive(SwerveModule... modules) {
        Objects.requireNonNull(modules);
        if (modules.length < 2) {
            throw new IllegalArgumentException(
                    "Swerve drive requires at least 2 wheels");
        }
        for (SwerveModule m : modules) {
            Objects.requireNonNull(m);
        }
        this.modules = modules.clone();

        Translation2d[] wheelLocations = new Translation2d[modules.length];
        for (int i = 0; i < wheelLocations.length; i++) {
            wheelLocations[i] = modules[i].getLocation();
        }
        kinematics = new SwerveDriveKinematics(wheelLocations);

        centerOfRotation = new Translation2d();

    }

    public void initializeHardware() {
        for (SwerveModule module : modules) {
            module.initializeHardware();
        }
    }

    public void setCenterOfRotation(Translation2d newCenterOfRotation) {
        centerOfRotation = newCenterOfRotation;
    }

    protected abstract Rotation2d getGyroAngle();

    public final void driveVelocity(double vx, double vy, double vr) {
        driveVelocity(vx, vy, vr, centerOfRotation);
    }

    public void driveVelocity(double vx, double vy, double vr,
            Translation2d centerOfRotation) {

    }
}
