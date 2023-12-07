package org.victorrobotics.dtlib.subsystem.swerve;

import static java.util.Objects.requireNonNull;

import org.victorrobotics.dtlib.DTRobot;
import org.victorrobotics.dtlib.hardware.IMU;
import org.victorrobotics.dtlib.math.geometry.Vector2D;
import org.victorrobotics.dtlib.math.geometry.Vector2D_R;
import org.victorrobotics.dtlib.math.kinematics.SwerveDriveKinematics;
import org.victorrobotics.dtlib.math.kinematics.SwerveDriveOdometry;
import org.victorrobotics.dtlib.math.kinematics.SwerveModulePosition;
import org.victorrobotics.dtlib.math.kinematics.SwerveModuleState;
import org.victorrobotics.dtlib.math.trajectory.AccelerationLimit;
import org.victorrobotics.dtlib.math.trajectory.VelocityLimit;
import org.victorrobotics.dtlib.subsystem.Subsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.function.IntFunction;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveDrive extends Subsystem {
  public static class Builder {
    private List<SwerveModule> modules;
    private IMU                imu;

    private AccelerationLimit accelLimit    = new AccelerationLimit();
    private VelocityLimit     velocityLimit = new VelocityLimit();

    Builder() {
      modules = new ArrayList<>();
    }

    public SwerveDrive build() {
      return new SwerveDrive(modules.toArray(SwerveModule[]::new), imu, velocityLimit, accelLimit);
    }

    public Builder addModule(SwerveModule module) {
      modules.add(requireNonNull(module));
      return this;
    }

    public Builder addModules(int count, IntFunction<SwerveModule> generator) {
      for (int i = 0; i < count; i++) {
        modules.add(requireNonNull(generator.apply(i)));
      }
      return this;
    }

    public Builder withIMU(IMU imu) {
      this.imu = imu;
      return this;
    }

    public Builder withAccelLimit(AccelerationLimit accelLimit) {
      this.accelLimit = accelLimit;
      return this;
    }

    public Builder withVelocityLimit(VelocityLimit velocityLimit) {
      this.velocityLimit = velocityLimit;
      return this;
    }
  }

  private static final double HALF_PI = Math.PI / 2;

  private final IMU            imu;
  private final SwerveModule[] modules;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry   odometry;

  private final SwerveModulePosition[] positions;

  private AccelerationLimit accelLimit;
  private VelocityLimit     velocityLimit;
  private boolean           isFieldRelative;

  private Vector2D_R currentSpeeds;

  public SwerveDrive(SwerveModule[] modules, IMU imu, VelocityLimit velocityLimit,
                     AccelerationLimit accelLimit) {
    requireNonNull(modules);

    this.imu = requireNonNull(imu);
    this.modules = new SwerveModule[modules.length];
    this.accelLimit = accelLimit;
    this.velocityLimit = velocityLimit;

    positions = new SwerveModulePosition[modules.length];

    Vector2D[] wheelLocations = new Vector2D[modules.length];
    double maxModuleSpeed = Double.POSITIVE_INFINITY;
    for (int i = 0; i < modules.length; i++) {
      this.modules[i] = requireNonNull(modules[i]);
      positions[i] = new SwerveModulePosition(modules[i].getDistance(), modules[i].getAngle());
      wheelLocations[i] = modules[i].getLocation();
      maxModuleSpeed = Math.min(maxModuleSpeed, modules[i].maxSpeed());
    }

    kinematics = new SwerveDriveKinematics(maxModuleSpeed, wheelLocations);
    odometry = new SwerveDriveOdometry(kinematics, new Vector2D_R(), getGyroAngle(), positions,
                                       new double[] { 0.1, 0.1, 0.1 });

    currentSpeeds = new Vector2D_R();
  }

  public void setCenterOfRotation(Vector2D newCenterOfRotation) {
    kinematics.setCenterOfRotation(newCenterOfRotation);
  }

  public void configFieldRelative(boolean isFieldRelative) {
    this.isFieldRelative = isFieldRelative;
  }

  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(imu.getYaw());
  }

  public void driveVelocity(double vx, double vy, double vr) {
    if (isFieldRelative) {
      double vxField = vx;
      double vyField = vy;

      Rotation2d invGyro = getGyroAngle().unaryMinus();
      vx = vxField * invGyro.getCos() - vyField * invGyro.getSin();
      vy = vxField * invGyro.getSin() + vyField * invGyro.getCos();
    }

    Vector2D_R previousSpeeds = currentSpeeds;
    currentSpeeds.setX(vx);
    currentSpeeds.setY(vy);
    currentSpeeds.setR(vr);

    velocityLimit.apply(currentSpeeds);
    accelLimit.apply(currentSpeeds, previousSpeeds);

    SwerveModuleState[] moduleStates = kinematics.computeModuleStates(currentSpeeds);
    for (int i = 0; i < modules.length; i++) {
      modules[i].setSpeed(moduleStates[i].speed);
      modules[i].setAngle(moduleStates[i].heading);
    }
  }

  public void holdPosition() {
    for (SwerveModule module : modules) {
      double angle = module.getLocation()
                           .theta();

      double currentAngle = module.getAngle()
                                  .getRadians();
      while (angle - currentAngle > HALF_PI) {
        angle -= Math.PI;
      }
      while (currentAngle - angle > HALF_PI) {
        angle += Math.PI;
      }

      module.setAngle(Rotation2d.fromRadians(angle));
      module.setSpeed(0);
    }
  }

  public Vector2D_R getEstimatedPose() {
    return odometry.getEstimatedPosition();
  }

  public SwerveDriveOdometry getOdometry() {
    return odometry;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < modules.length; i++) {
      positions[i].angle = modules[i].getAngle();
      positions[i].distance = modules[i].getDistance();
    }
    odometry.update(DTRobot.currentTime(), getGyroAngle(), positions);
  }

  public static Builder builder() {
    return new Builder();
  }
}
