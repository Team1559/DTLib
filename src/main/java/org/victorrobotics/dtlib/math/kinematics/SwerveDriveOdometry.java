package org.victorrobotics.dtlib.math.kinematics;

import java.util.Arrays;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;

public class SwerveDriveOdometry {
  private class InterpolationRecord implements Interpolatable<InterpolationRecord> {
    private final Pose2d                 pose;
    private final Rotation2d             gyroAngle;
    private final SwerveModulePosition[] wheelPositions;

    InterpolationRecord(Pose2d pose, Rotation2d gyro, SwerveModulePosition... wheelPositions) {
      this.pose = pose;
      this.gyroAngle = gyro;
      this.wheelPositions = deepCopy(wheelPositions, null);
    }

    @Override
    public InterpolationRecord interpolate(InterpolationRecord endValue, double t) {
      if (t < 0) return this;
      if (t >= 1) return endValue;

      SwerveModulePosition[] wheelLerp = new SwerveModulePosition[wheelPositions.length];
      for (int i = 0; i < wheelPositions.length; i++) {
        wheelLerp[i] = wheelPositions[i].interpolate(endValue.wheelPositions[i], t);
      }

      Rotation2d gyroLerp = gyroAngle.interpolate(endValue.gyroAngle, t);

      Twist2d twist = kinematics.computeDeltaPose(wheelPositions, wheelLerp);
      twist.dtheta = gyroLerp.minus(gyroAngle)
                             .getRadians();

      return new InterpolationRecord(pose.exp(twist), gyroLerp, wheelLerp);
    }

    @Override
    public boolean equals(Object obj) {
      return this == obj || (obj instanceof InterpolationRecord other
          && Objects.equals(pose, other.pose) && Objects.equals(gyroAngle, other.gyroAngle)
          && Arrays.equals(wheelPositions, other.wheelPositions));
    }

    @Override
    public int hashCode() {
      return Objects.hash(pose, gyroAngle, wheelPositions);
    }
  }

  private static final double BUFFER_DURATION = 1.5;

  private final TimeInterpolatableBuffer<InterpolationRecord> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(BUFFER_DURATION);

  private final SwerveDriveKinematics  kinematics;
  private final SwerveModulePosition[] modulePositions;

  private final double[] encoderStdDevsSquared;
  private final double[] visionKalmanFilter;

  private final int moduleCount;

  private Pose2d     estimatedPose;
  private Rotation2d gyroOffset;

  public SwerveDriveOdometry(SwerveDriveKinematics kinematics, Pose2d initialPose,
                             Rotation2d gyroAngle, SwerveModulePosition[] modulePositions,
                             double[] encoderStdDevs) {
    this.kinematics = kinematics;
    this.moduleCount = modulePositions.length;
    this.modulePositions = deepCopy(modulePositions, null);

    estimatedPose = initialPose;
    gyroOffset = estimatedPose.getRotation()
                              .minus(gyroAngle);

    visionKalmanFilter = new double[3];
    encoderStdDevsSquared = new double[3];
    for (int i = 0; i < 3; ++i) {
      encoderStdDevsSquared[i] = encoderStdDevs[i] * encoderStdDevs[i];
    }
  }

  /**
   * @return The estimated robot position.
   */
  public Pose2d getEstimatedPosition() {
    return estimatedPose;
  }

  public void resetPosition(Pose2d newPose, Rotation2d gyroAngle,
                            SwerveModulePosition... modulePositions) {
    if (modulePositions.length != moduleCount) {
      throwInconsistentModuleCount();
    }

    estimatedPose = newPose;
    gyroOffset = estimatedPose.getRotation()
                              .minus(gyroAngle);
    deepCopy(modulePositions, this.modulePositions);
    poseBuffer.clear();
  }

  public void addVisionMeasurement(Pose2d robotPose, double timeSeconds, double... visionStdDevs) {
    // If measurement is too old, ignore
    try {
      double newestEntryTime = poseBuffer.getInternalBuffer()
                                         .lastKey();
      if (newestEntryTime - BUFFER_DURATION > timeSeconds) return;
    } catch (NoSuchElementException ex) {
      return;
    }

    // Get the estimated pose at the moment the vision measurement was made
    Optional<InterpolationRecord> sampleOptional = poseBuffer.getSample(timeSeconds);
    if (sampleOptional.isEmpty()) return;
    InterpolationRecord sample = sampleOptional.get();

    // Measure the difference between the odometry pose and the vision pose
    Twist2d twist = sample.pose.log(robotPose);

    // Scale by how much we trust vision compared to encoders
    // https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/algorithms.md
    for (int i = 0; i < 3; i++) {
      visionKalmanFilter[i] = encoderStdDevsSquared[i] < 1e-6 ? 0
          : (encoderStdDevsSquared[i] / (encoderStdDevsSquared[i]
              + Math.sqrt(encoderStdDevsSquared[i] * visionStdDevs[i] * visionStdDevs[i])));
    }
    Twist2d scaledTwist =
        new Twist2d(visionKalmanFilter[0] * twist.dx, visionKalmanFilter[1] * twist.dy,
                    visionKalmanFilter[2] * twist.dtheta);

    // Reset state to time of sample, with the new vision adjustment
    resetPosition(sample.pose.exp(scaledTwist), sample.gyroAngle, sample.wheelPositions);

    // Inject new pose at given timestamp
    InterpolationRecord newSample =
        new InterpolationRecord(estimatedPose, sample.gyroAngle, sample.wheelPositions);
    poseBuffer.addSample(timeSeconds, newSample);

    // Replay all odometry inputs since the vision measurement
    for (Map.Entry<Double, InterpolationRecord> input : poseBuffer.getInternalBuffer()
                                                                  .tailMap(timeSeconds)
                                                                  .entrySet()) {
      update(input.getKey(), input.getValue().gyroAngle, input.getValue().wheelPositions);
    }
  }

  public void update(double timeSeconds, Rotation2d gyroAngle,
                     SwerveModulePosition[] modulePositions) {
    if (modulePositions.length != moduleCount) {
      throwInconsistentModuleCount();
    }

    Rotation2d angle = gyroAngle.plus(gyroOffset);
    Twist2d twist = kinematics.computeDeltaPose(this.modulePositions, modulePositions);
    twist.dtheta = angle.minus(estimatedPose.getRotation())
                        .getRadians();

    deepCopy(modulePositions, this.modulePositions);
    estimatedPose = new Pose2d(estimatedPose.exp(twist)
                                            .getTranslation(),
                               angle);

    poseBuffer.addSample(timeSeconds,
                         new InterpolationRecord(estimatedPose, gyroAngle, modulePositions));
  }

  private static SwerveModulePosition[] deepCopy(SwerveModulePosition[] src,
                                                 SwerveModulePosition[] dest) {
    if (dest == null) {
      dest = new SwerveModulePosition[src.length];
    }

    for (int i = 0; i < src.length; i++) {
      dest[i] = src[i].clone();
    }
    return dest;
  }

  private static void throwInconsistentModuleCount() {
    throw new IllegalArgumentException("Inconsistent number of modules");
  }
}
