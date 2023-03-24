package org.victorrobotics.frc.dtlib.drivetrain.swerve.trajectory;

import org.victorrobotics.frc.dtlib.drivetrain.DTAccelerationLimit;
import org.victorrobotics.frc.dtlib.drivetrain.DTVelocityLimit;
import org.victorrobotics.frc.dtlib.drivetrain.swerve.trajectory.DTSwerveTrajectory.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
        double[][] interpolatedPoints = interpolate(waypoints);
        double[][] smoothPoints = smooth(interpolatedPoints);

        Pose2d[] path = new Pose2d[smoothPoints.length];
        for (int i = 0; i < path.length; i++) {
            path[i] = new Pose2d(smoothPoints[i][0], smoothPoints[i][1],
                    Rotation2d.fromDegrees(smoothPoints[i][2]));
        }

        // cumulative time, cumulative distance, commanded velocity, predicted
        // velocity, linear acceleration, curvature
        double[][] dataValues = new double[6][path.length];
        computeVelocity(path, dataValues[1], dataValues[2], dataValues[3], dataValues[5]);
        computeAcceleration(dataValues[1], dataValues[3], dataValues[0], dataValues[4]);

        Point[] points = constructPoints(path, dataValues[0], dataValues[1], dataValues[2],
                dataValues[3], dataValues[4], dataValues[5]);
        return new DTSwerveTrajectory(points);
    }

    private double[][] interpolate(Pose2d[] waypoints) {
        // Calculate how many points to insert into each segment and their
        // initial transforms
        int[] intermediatePointCounts = new int[waypoints.length - 1];
        double[][] deltas = new double[waypoints.length - 1][3];
        int totalPointCount = 1;
        for (int i = 0; i < deltas.length; i++) {
            Pose2d currentPoint = waypoints[i];
            Pose2d nextPoint = waypoints[i + 1];
            deltas[i][0] = nextPoint.getX() - currentPoint.getX();
            deltas[i][1] = nextPoint.getY() - currentPoint.getY();
            deltas[i][2] = nextPoint.getRotation()
                                    .getDegrees()
                    - currentPoint.getRotation()
                                  .getDegrees();

            double deltaD = Math.hypot(deltas[i][0], deltas[i][1]);
            int pointCount = (int) (deltaD / distanceBetweenPoints);
            deltas[i][0] /= pointCount;
            deltas[i][1] /= pointCount;
            deltas[i][2] /= pointCount;
            intermediatePointCounts[i] = pointCount - 1;
            totalPointCount += pointCount;
        }
        // Insert points by adding the deltas
        // Use 2D double array for less object creation and destruction during
        // editing
        double[][] points = new double[totalPointCount][3];
        int pointIndex = 0;
        double[] currentPoint = points[pointIndex];
        double x, y, r, dx, dy, dr;
        for (int i = 0; i < deltas.length; i++) {
            // Copy original point
            x = waypoints[i].getX();
            y = waypoints[i].getY();
            r = waypoints[i].getRotation()
                            .getDegrees();
            currentPoint[0] = x;
            currentPoint[1] = y;
            currentPoint[2] = r;
            pointIndex++;
            currentPoint = points[pointIndex];
            // Create intermediate points
            dx = deltas[i][0];
            dy = deltas[i][1];
            dr = deltas[i][2];
            for (int j = 1; j <= intermediatePointCounts[i]; j++) {
                currentPoint[0] = x + dx * j;
                currentPoint[1] = y + dy * j;
                currentPoint[2] = r + dr * j;
                pointIndex++;
                currentPoint = points[pointIndex];
            }
        }
        double[] lastPoint = points[points.length - 1];
        lastPoint[0] = waypoints[waypoints.length - 1].getX();
        lastPoint[1] = waypoints[waypoints.length - 1].getY();
        lastPoint[2] = waypoints[waypoints.length - 1].getRotation()
                                                      .getDegrees();
        return points;
    }

    private double[][] smooth(double[][] points) {
        double[][] newPoints = new double[points.length][points[0].length];
        for (int i = 0; i < newPoints.length; i++) {
            System.arraycopy(points[i], 0, newPoints[i], 0, points[i].length);
        }

        double pathWeight = 1 - smoothWeight;
        double change = SMOOTH_TOLERANCE;
        while (change >= SMOOTH_TOLERANCE) {
            change = 0;
            for (int i = 1; i < newPoints.length - 1; i++) {
                double deltaX = pathWeight * (points[i][0] - newPoints[i][0]) + smoothWeight
                        * (newPoints[i - 1][0] + newPoints[i + 1][0] - 2 * newPoints[i][0]);
                newPoints[i][0] += deltaX;

                double deltaY = pathWeight * (points[i][1] - newPoints[i][1]) + smoothWeight
                        * (newPoints[i - 1][1] + newPoints[i + 1][1] - 2 * newPoints[i][1]);
                newPoints[i][1] += deltaY;

                change += Math.hypot(deltaX, deltaY);
            }
        }

        return newPoints;
    }

    private void computeVelocity(Pose2d[] path, double[] distance, double[] commandVelocity,
            double[] predictVelocity, double[] curvature) {
        // Compute the commanded velocities and cumulative distances
        double maxLinearVelocity = velocityLimit.maxVelocityTranslation;
        double maxLinearAccel = accelerationLimit.maxAccelTranslation;
        for (int i = 1; i < path.length - 1; i++) {
            distance[i] = distance[i - 1] + path[i].minus(path[i - 1])
                                                   .getTranslation()
                                                   .getNorm();
            curvature[i] = calculateCurvature(path[i - 1], path[i], path[i + 1]);
            if (Double.isNaN(curvature[i])) {
                // Turn around in place
                commandVelocity[i] = 0;
            } else {
                commandVelocity[i] = Math.min(
                        velocityCoefficient / Math.pow(curvature[i], velocityPower),
                        maxLinearVelocity);
            }
        }
        commandVelocity[0] = maxLinearVelocity;
        distance[path.length - 1] = distance[path.length - 2]
                + path[path.length - 1].minus(path[path.length - 2])
                                       .getTranslation()
                                       .getNorm();
        // Apply deceleration where needed
        for (int i = path.length - 2; i >= 0; i--) {
            double newVelocity = Math.sqrt(commandVelocity[i + 1] * commandVelocity[i + 1]
                    + 2 * maxLinearAccel * (distance[i + 1] - distance[i]));
            if (!Double.isNaN(newVelocity) && newVelocity < commandVelocity[i]) {
                commandVelocity[i] = newVelocity;
            }
        }
        /*
         * To project the actual performance of the robot, the commanded
         * velocities are turned into predicted velocities
         */
        System.arraycopy(commandVelocity, 0, predictVelocity, 0, path.length);
        predictVelocity[0] = 0;
        for (int i = 1; i < path.length - 1; i++) {
            double newVelocity = Math.sqrt(predictVelocity[i - 1] * predictVelocity[i - 1]
                    + 2 * maxLinearAccel * (distance[i] - distance[i - 1]));
            if (!Double.isNaN(newVelocity) && newVelocity < commandVelocity[i]) {
                predictVelocity[i] = newVelocity;
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

    private static void computeAcceleration(double[] distances, double[] predictedVelocities,
            double[] times, double[] accelerations) {
        double time = 0;
        for (int i = 1; i < times.length; i++) {
            double avgVelocity = 0.5 * (predictedVelocities[i] + predictedVelocities[i - 1]);
            double distance = distances[i] - distances[i - 1];
            time += distance / avgVelocity;
            times[i] = time;

            double dv = predictedVelocities[i] - predictedVelocities[i - 1];
            double dt = times[i] - times[i - 1];
            accelerations[i - 1] = dv / dt;
        }
    }

    private static Point[] constructPoints(Pose2d[] path, double[] time, double[] distance,
            double[] commandVelocity, double[] predictVelocity, double[] acceleration,
            double[] curvature) {
        Point[] points = new Point[path.length];
        for (int i = 0; i < path.length; i++) {
            points[i] = new Point(path[i], time[i], distance[i], commandVelocity[i],
                    predictVelocity[i], acceleration[i], curvature[i]);
        }
        return points;
    }
}
