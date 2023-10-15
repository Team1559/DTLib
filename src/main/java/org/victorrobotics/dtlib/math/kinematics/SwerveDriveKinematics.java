package org.victorrobotics.dtlib.math.kinematics;

import org.victorrobotics.dtlib.math.geometry.Vector2D;
import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

/**
 * Helper class that converts a chassis velocity (dx, dy, and dtheta components)
 * into individual module states (speed and angle).
 * <p>
 * The inverse kinematics (converting from a desired chassis velocity to
 * individual module states) uses the relative locations of the modules with
 * respect to the center of rotation. The center of rotation for inverse
 * kinematics is also variable. This means that you can set your center of
 * rotation in a corner of the robot to perform special evasion maneuvers.
 * <p>
 * Forward kinematics (converting an array of module states into the overall
 * chassis motion) is performs the exact opposite of what inverse kinematics
 * does. Since this is an overdetermined system (more equations than variables),
 * we use a least-squares approximation.
 * <p>
 * The inverse kinematics: [moduleStates] = [moduleLocations] * [chassisSpeeds]
 * We take the Moore-Penrose pseudoinverse of [moduleLocations] and then
 * multiply by [moduleStates] to get our chassis speeds.
 * <p>
 * Forward kinematics is also used for odometry -- determining the position of
 * the robot on the field using encoders and a gyro.
 */
public class SwerveDriveKinematics {
  private static final Rotation2d DEGREES_180 = Rotation2d.fromDegrees(180);

  private final DMatrixRMaj inverseMatrix;
  private final DMatrixRMaj forwardMatrix;

  private final DMatrixRMaj chassisSpeedsMatrix;
  private final DMatrixRMaj moduleStatesMatrix;

  private final int          moduleCount;
  private final Vector2D[]   moduleLocations;
  private final Rotation2d[] moduleHeadings;

  private final double maxModuleSpeed;

  /**
   * Constructs a swerve drive kinematics object. This takes in a variable
   * number of module locations as Translation2d objects. The order in which you
   * pass in the module locations is the same order that you will receive the
   * module states when performing inverse kinematics. It is also expected that
   * you pass in the module states in the same order when calling the forward
   * kinematics methods.
   *
   * @param moduleLocations
   *        The locations of the modules relative to the physical center of the
   *        robot.
   */
  public SwerveDriveKinematics(double maxModuleSpeed, Vector2D... moduleLocations) {
    if (moduleLocations.length < 2) {
      throw new IllegalArgumentException("Swerve drive requires at least two modules");
    }

    this.maxModuleSpeed = maxModuleSpeed;
    this.moduleCount = moduleLocations.length;
    this.moduleLocations = new Vector2D[moduleCount];
    for (int i = 0; i < moduleCount; i++) {
      this.moduleLocations[i] = moduleLocations[i].clone();
    }

    moduleHeadings = new Rotation2d[moduleCount];
    Arrays.fill(moduleHeadings, new Rotation2d());

    moduleStatesMatrix = new DMatrixRMaj(moduleCount * 2, 1);
    chassisSpeedsMatrix = new DMatrixRMaj(3, 1);
    inverseMatrix = new DMatrixRMaj(moduleCount * 2, 3);
    forwardMatrix = new DMatrixRMaj(3, moduleCount * 2);

    for (int i = 0; i < moduleCount; i++) {
      inverseMatrix.set(i * 2, 0, 1);
      inverseMatrix.set(i * 2, 1, 0);
      inverseMatrix.set(i * 2, 2, -moduleLocations[i].getY());
      inverseMatrix.set(i * 2 + 1, 0, 0);
      inverseMatrix.set(i * 2 + 1, 1, 1);
      inverseMatrix.set(i * 2 + 1, 2, moduleLocations[i].getX());
    }
    CommonOps_DDRM.pinv(inverseMatrix, forwardMatrix);
  }

  /**
   * Reset the internal swerve module headings.
   *
   * @param moduleHeadings
   *        The swerve module headings. The order of the module headings should
   *        be same as passed into the constructor of this class.
   */
  public void updateModuleHeadings(Rotation2d... moduleHeadings) {
    if (moduleHeadings.length != moduleCount) {
      throwInconsistentModuleCount();
    }
    System.arraycopy(moduleHeadings, 0, this.moduleHeadings, 0, moduleCount);
  }

  /**
   * Performs inverse kinematics to return the module states from a desired
   * chassis velocity. This method is often used to convert joystick values into
   * module speeds and angles.
   * <p>
   * This function also supports variable centers of rotation. During normal
   * operations, the center of rotation is usually the same as the physical
   * center of the robot; therefore, the argument is defaulted to that use case.
   * However, if you wish to change the center of rotation for evasive
   * maneuvers, vision alignment, or for any other use case, you can do so.
   * <p>
   * In the case that the desired chassis speeds are zero (i.e. the robot will
   * be stationary), the previously calculated module angle will be maintained.
   *
   * @param robotVelocity
   *        The desired chassis speed.
   *
   * @return An array containing the module states. Use caution because these
   *         module states are not normalized. Sometimes, a user input may cause
   *         one of the module speeds to go above the attainable max velocity.
   *         Use the {@link #desaturateWheelSpeeds(SwerveModuleState[], double)
   *         DesaturateWheelSpeeds} function to rectify this issue.
   */
  @SuppressWarnings("java:S3518") // "possible" divide by zero
  public SwerveModuleState[] computeModuleStates(Vector2D_R robotVelocity) {
    SwerveModuleState[] moduleStates = new SwerveModuleState[moduleCount];

    if (Math.abs(robotVelocity.getX()) < 1e-6 && Math.abs(robotVelocity.getY()) < 1e-6
        && Math.abs(robotVelocity.getR()) < 1e-6) {
      for (int i = 0; i < moduleCount; i++) {
        moduleStates[i] = new SwerveModuleState(0, moduleHeadings[i]);
      }
      return moduleStates;
    }

    chassisSpeedsMatrix.set(0, 0, robotVelocity.getX());
    chassisSpeedsMatrix.set(1, 0, robotVelocity.getY());
    chassisSpeedsMatrix.set(2, 0, robotVelocity.getR());
    CommonOps_DDRM.mult(inverseMatrix, chassisSpeedsMatrix, moduleStatesMatrix);

    double maxSpeed = 0;
    for (int i = 0; i < moduleCount; i++) {
      double x = moduleStatesMatrix.get(i * 2, 0);
      double y = moduleStatesMatrix.get(i * 2 + 1, 0);
      double speed = Math.hypot(x, y);
      Rotation2d angle = new Rotation2d(x, y);

      if (speed > maxSpeed) {
        maxSpeed = speed;
      }

      // Optimize heading
      Rotation2d delta = angle.minus(moduleHeadings[i]);
      if (Math.abs(delta.getDegrees()) >= 90) {
        speed = -speed;
        angle = angle.rotateBy(DEGREES_180);
      }

      moduleStates[i] = new SwerveModuleState(speed, angle);
      moduleHeadings[i] = moduleStates[i].angle;
    }

    // Scale down speeds if needed
    if (maxSpeed > maxModuleSpeed) {
      double scale = maxModuleSpeed / maxSpeed;
      for (SwerveModuleState moduleState : moduleStates) {
        moduleState.speed *= scale;
      }
    }

    return moduleStates;
  }

  /**
   * Performs forward kinematics to return the resulting chassis state from the
   * given module states. This method is often used for odometry -- determining
   * the robot's position on the field using data from the real-world speed and
   * angle of each module on the robot.
   *
   * @param moduleStates
   *        The state of the modules (as a SwerveModuleState type) as measured
   *        from respective encoders and gyros. The order of the swerve module
   *        states should be same as passed into the constructor of this class.
   *
   * @return The resulting chassis speed.
   */
  public Vector2D_R computeRobotVelocity(SwerveModuleState... moduleStates) {
    if (moduleStates.length != moduleCount) {
      throwInconsistentModuleCount();
    }

    for (int i = 0; i < moduleCount; i++) {
      SwerveModuleState module = moduleStates[i];
      moduleStatesMatrix.set(i * 2, 0, module.speed * module.angle.getCos());
      moduleStatesMatrix.set(i * 2 + 1, module.speed * module.angle.getSin());
    }

    CommonOps_DDRM.mult(forwardMatrix, moduleStatesMatrix, chassisSpeedsMatrix);
    return new Vector2D_R(chassisSpeedsMatrix.get(0, 0), chassisSpeedsMatrix.get(1, 0),
                          chassisSpeedsMatrix.get(2, 0));
  }

  /**
   * Performs forward kinematics to return the resulting Twist2d from the given
   * change in wheel positions. This method is often used for odometry --
   * determining the robot's position on the field using changes in the distance
   * driven by each wheel on the robot.
   *
   * @param start
   *        The starting distances driven by the wheels.
   * @param end
   *        The ending distances driven by the wheels.
   *
   * @return The resulting Twist2d in the robot's movement.
   */
  public Twist2d computeDeltaPose(SwerveModulePosition[] start, SwerveModulePosition[] end) {
    if (start.length != end.length) {
      throwInconsistentModuleCount();
    }

    for (int i = 0; i < start.length; i++) {
      double distance = end[i].distance - start[i].distance;
      Rotation2d angle = end[i].angle;
      moduleStatesMatrix.set(i * 2, 0, distance * angle.getCos());
      moduleStatesMatrix.set(i * 2 + 1, 0, distance * angle.getSin());
    }

    CommonOps_DDRM.mult(forwardMatrix, moduleStatesMatrix, chassisSpeedsMatrix);
    return new Twist2d(chassisSpeedsMatrix.get(0, 0), chassisSpeedsMatrix.get(1, 0),
                       chassisSpeedsMatrix.get(2, 0));
  }

  public void setCenterOfRotation(Vector2D centerOfRotation) {
    for (int i = 0; i < moduleCount; i++) {
      inverseMatrix.set(i * 2, 0, 1);
      inverseMatrix.set(i * 2, 1, 0);
      inverseMatrix.set(i * 2, 2, -moduleLocations[i].getY() + centerOfRotation.getY());
      inverseMatrix.set(i * 2 + 1, 0, 0);
      inverseMatrix.set(i * 2 + 1, 1, 1);
      inverseMatrix.set(i * 2 + 1, 2, moduleLocations[i].getX() - centerOfRotation.getX());
    }
  }

  private static void throwInconsistentModuleCount() {
    throw new IllegalArgumentException("Inconsistent number of modules");
  }
}
