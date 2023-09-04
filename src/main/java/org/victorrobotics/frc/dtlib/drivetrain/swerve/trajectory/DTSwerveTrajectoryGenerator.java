package org.victorrobotics.frc.dtlib.drivetrain.swerve.trajectory;

import org.victorrobotics.frc.dtlib.drivetrain.DTAccelerationLimit;
import org.victorrobotics.frc.dtlib.drivetrain.DTVelocityLimit;
import org.victorrobotics.frc.dtlib.math.geometry.DTMutablePose2d;

import edu.wpi.first.math.geometry.Pose2d;

public class DTSwerveTrajectoryGenerator {
  private static final double SMOOTH_TOLERANCE = 0.005;

  private static final DTAccelerationLimit DEFAULT_ACCEL_LIMIT = new DTAccelerationLimit();
  private static final DTVelocityLimit     DEFAULT_VELO_LIMIT  = new DTVelocityLimit();

  private static final double DEFAULT_DISTANCE_BETWEEN_POINTS = 0.05;
  private static final double DEFAULT_SMOOTH_WEIGHT           = 0.75;
  private static final double DEFAULT_VELOCITY_POWER          = 0.2;
  private static final double DEFAULT_VELOCITY_COEFFICIENT    = 5;

  private DTAccelerationLimit accelerationLimit;
  private DTVelocityLimit     velocityLimit;

  private double distanceBetweenPoints;
  private double smoothWeight;
  private double velocityPower;
  private double velocityCoefficient;

  public DTSwerveTrajectoryGenerator() {
    distanceBetweenPoints = DEFAULT_DISTANCE_BETWEEN_POINTS;
    smoothWeight = DEFAULT_SMOOTH_WEIGHT;
    velocityPower = DEFAULT_VELOCITY_POWER;
    velocityCoefficient = DEFAULT_VELOCITY_COEFFICIENT;

    accelerationLimit = DEFAULT_ACCEL_LIMIT;
    velocityLimit = DEFAULT_VELO_LIMIT;
  }

  public void setAccelerationLimit(DTAccelerationLimit limit) {
    accelerationLimit = limit;
  }

  public void setVelocityLimit(DTVelocityLimit limit) {
    velocityLimit = limit;
  }

  public void setDistanceBetweenPoints(double distanceMeters) {
    distanceBetweenPoints = distanceMeters;
  }

  public void setSmoothWeight(double weight) {
    smoothWeight = weight;
  }

  public void setVelocityPower(double power) {
    velocityPower = power;
  }

  public void setVelocityCoefficient(double coefficient) {
    velocityCoefficient = coefficient;
  }

  public DTSwerveTrajectory generate(Pose2d... waypoints) {
    DTMutablePose2d[] interpolatedPoints = interpolate(waypoints);
    smooth(interpolatedPoints);

    DTSwerveTrajectory.Point[] path = new DTSwerveTrajectory.Point[interpolatedPoints.length];
    for (int i = 0; i < path.length; i++) {
      path[i] = new DTSwerveTrajectory.Point();
      path[i].pose = interpolatedPoints[i].toPose2d();
    }

    computeVelocity(path);
    computeAcceleration(path);

    return new DTSwerveTrajectory(path);
  }

  private DTMutablePose2d[] interpolate(Pose2d[] waypoints) {
    // Calculate how many points to insert into each segment and their
    // initial transforms
    int[] intermediatePointCounts = new int[waypoints.length - 1];
    DTMutablePose2d[] deltas = new DTMutablePose2d[waypoints.length - 1];
    int totalPointCount = 1;
    for (int i = 0; i < deltas.length; i++) {
      Pose2d currentPoint = waypoints[i];
      Pose2d nextPoint = waypoints[i + 1];
      deltas[i].x = nextPoint.getX() - currentPoint.getX();
      deltas[i].y = nextPoint.getY() - currentPoint.getY();
      deltas[i].r = nextPoint.getRotation()
                             .minus(currentPoint.getRotation())
                             .getDegrees();
      double displacement = Math.hypot(deltas[i].x, deltas[i].y);
      int pointCount = (int) (displacement / distanceBetweenPoints);
      deltas[i].x /= pointCount;
      deltas[i].y /= pointCount;
      deltas[i].r /= pointCount;
      intermediatePointCounts[i] = pointCount - 1;
      totalPointCount += pointCount;
    }
    // Insert points by summing the deltas
    DTMutablePose2d[] points = new DTMutablePose2d[totalPointCount];
    int pointIndex = 0;
    DTMutablePose2d currentPoint = points[pointIndex];
    double x, y, r, dx, dy, dr;
    for (int i = 0; i < deltas.length; i++) {
      // Copy original point
      x = waypoints[i].getX();
      y = waypoints[i].getY();
      r = waypoints[i].getRotation()
                      .getDegrees();
      currentPoint.x = x;
      currentPoint.y = y;
      currentPoint.r = r;
      pointIndex++;
      currentPoint = points[pointIndex];
      // Create intermediate points
      dx = deltas[i].x;
      dy = deltas[i].y;
      dr = deltas[i].r;
      for (int j = 1; j <= intermediatePointCounts[i]; j++) {
        currentPoint.x = x + dx * j;
        currentPoint.y = y + dy * j;
        currentPoint.r = r + dr * j;
        pointIndex++;
        currentPoint = points[pointIndex];
      }
    }
    DTMutablePose2d lastPoint = points[points.length - 1];
    lastPoint.x = waypoints[waypoints.length - 1].getX();
    lastPoint.y = waypoints[waypoints.length - 1].getY();
    lastPoint.r = waypoints[waypoints.length - 1].getRotation()
                                                 .getDegrees();
    return points;
  }

  private void smooth(DTMutablePose2d[] points) {
    DTMutablePose2d[] newPoints = new DTMutablePose2d[points.length];
    for (int i = 0; i < points.length; i++) {
      newPoints[i] = points[i].clone();
    }

    double pathWeight = 1 - smoothWeight;
    double change = SMOOTH_TOLERANCE;
    while (change >= SMOOTH_TOLERANCE) {
      change = 0;
      for (int i = 1; i < newPoints.length - 1; i++) {
        double deltaX = pathWeight * (points[i].x - newPoints[i].x)
            + smoothWeight * (newPoints[i - 1].x + newPoints[i + 1].x - 2 * newPoints[i].x);
        newPoints[i].x += deltaX;

        double deltaY = pathWeight * (points[i].y - newPoints[i].y)
            + smoothWeight * (newPoints[i - 1].y + newPoints[i + 1].y - 2 * newPoints[i].y);
        newPoints[i].y += deltaY;

        change += Math.hypot(deltaX, deltaY);
      }
    }

    // Copy smoothed path to original array
    System.arraycopy(newPoints, 0, points, 0, points.length);
  }

  private void computeVelocity(DTSwerveTrajectory.Point[] path) {
    double maxLinearVelocity = velocityLimit.maxVelocityTranslation;
    double maxLinearAccel = accelerationLimit.maxAccelTranslation;
    for (int i = 1; i < path.length - 1; i++) {
      path[i].distance = path[i - 1].distance + path[i].pose.getTranslation()
                                                            .minus(
                                                                path[i - 1].pose.getTranslation())
                                                            .getNorm();
      path[i].curvature = calculateCurvature(path[i - 1].pose, path[i].pose, path[i + 1].pose);
      if (!Double.isFinite(path[i].curvature)) {
        // Turn around in place
        path[i].commandVelocity = 0;
      } else {
        path[i].commandVelocity = Math.min(
            velocityCoefficient / Math.pow(path[i].curvature, velocityPower), maxLinearVelocity);
      }
    }
    path[0].commandVelocity = maxLinearVelocity;
    // @format:off
        path[path.length - 1].distance = path[path.length - 2].distance
                + path[path.length - 1].pose.getTranslation()
                                            .minus(path[path.length - 2].pose.getTranslation())
                                            .getNorm();
        // @format:on

    // Apply deceleration where needed
    for (int i = path.length - 2; i >= 0; i--) {
      double newVelocity = Math.sqrt(path[i + 1].commandVelocity * path[i + 1].commandVelocity
          + 2 * maxLinearAccel * (path[i + 1].distance - path[i].distance));
      if (Double.isFinite(newVelocity) && newVelocity < path[i].commandVelocity) {
        path[i].commandVelocity = newVelocity;
      }
    }

    // To project the actual performance of the robot, the commanded
    // velocities are turned into predicted velocities with accel limits
    path[0].predictVelocity = 0;
    for (int i = 1; i < path.length - 1; i++) {
      double accelLimitedVelocity = Math.sqrt(
          path[i - 1].predictVelocity * path[i - 1].predictVelocity
              + 2 * maxLinearAccel * (path[i].distance - path[i - 1].distance));
      if (!Double.isNaN(accelLimitedVelocity) && accelLimitedVelocity < path[i].commandVelocity) {
        path[i].predictVelocity = accelLimitedVelocity;
      } else {
        path[i].predictVelocity = path[i].commandVelocity;
      }
    }
  }

  private static double calculateCurvature(Pose2d previous, Pose2d current, Pose2d next) {
    double x1 = previous.getX();
    double x2 = current.getX();
    double x3 = next.getX();
    double y1 = previous.getY();
    double y2 = current.getY();
    double y3 = next.getY();

    double dx1 = x1 - x2;
    double dx2 = x2 - x3;
    double dy1 = y1 - y2;
    double dy2 = y2 - y3;
    double a = dx1 * dy2 - dy1 * dx2;
    double b = dx1 * dx2 + dy1 * dy2;

    if (Math.abs(a) > 1e-9) {
      // Standard case
      return Math.abs(a / (b * Math.hypot(b, a)));
    }

    double dx3 = x3 - x1;
    double dy3 = y3 - y1;

    // Points are collinear, check for direction change
    double d12 = Math.hypot(dx1, dy1);
    double d23 = Math.hypot(dx2, dy2);
    double d13 = Math.hypot(dx3, dy3);
    if (d12 + d23 - d13 < 1e-6) {
      // 2 is between 1 and 3
      return 0;
    } else {
      // 2 is beyond 1 or 3
      return Double.NaN;
    }
  }

  private static void computeAcceleration(DTSwerveTrajectory.Point[] path) {
    double time = 0;
    for (int i = 1; i < path.length; i++) {
      double avgVelocity = 0.5 * (path[i].predictVelocity + path[i - 1].predictVelocity);
      double distance = path[i].distance - path[i - 1].distance;
      time += distance / avgVelocity;
      path[i].time = time;

      double dv = path[i].predictVelocity - path[i - 1].predictVelocity;
      double dt = path[i].time - path[i - 1].time;
      path[i - 1].acceleration = dv / dt;
    }
  }
}
