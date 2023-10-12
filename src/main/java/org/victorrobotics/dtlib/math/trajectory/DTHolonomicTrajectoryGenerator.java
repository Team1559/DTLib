package org.victorrobotics.dtlib.math.trajectory;

import org.victorrobotics.dtlib.math.geometry.DTVector2dR;
import org.victorrobotics.dtlib.math.spline.DTSpline;
import org.victorrobotics.dtlib.math.trajectory.DTHolonomicTrajectory.Constraint;
import org.victorrobotics.dtlib.math.trajectory.DTHolonomicTrajectory.Point;

import java.util.ArrayList;
import java.util.List;

public class DTHolonomicTrajectoryGenerator {
  private static final DTAccelerationLimit DEFAULT_ACCEL_LIMIT    =
      new DTAccelerationLimit(25, 20 * Math.PI);
  private static final DTVelocityLimit     DEFAULT_VELOCITY_LIMIT =
      new DTVelocityLimit(0.05, 5, 0.1, Math.PI * 4);

  private static final double DEFAULT_DISTANCE_BETWEEN_POINTS = 0.02;
  private static final int    DEFAULT_MIN_POINT_COUNT         = 8;

  private DTAccelerationLimit accelLimit;
  private DTVelocityLimit     velocityLimit;
  private double              distanceBetweenPoints;
  private double              deltaU;

  public DTHolonomicTrajectoryGenerator() {
    accelLimit = DEFAULT_ACCEL_LIMIT;
    velocityLimit = DEFAULT_VELOCITY_LIMIT;
    distanceBetweenPoints = DEFAULT_DISTANCE_BETWEEN_POINTS;
    deltaU = 1D / DEFAULT_MIN_POINT_COUNT;
  }

  public void setAccelLimit(DTAccelerationLimit limit) {
    accelLimit = limit;
  }

  public void setVelocityLimit(DTVelocityLimit limit) {
    velocityLimit = limit;
  }

  public void setDistanceBetweenPoints(double distanceMeters) {
    distanceBetweenPoints = distanceMeters;
  }

  public void setMinPointCount(int count) {
    deltaU = 1D / count;
  }

  public DTHolonomicTrajectory generate(DTSpline<?> spline) {
    Point[] points = generatePoints(spline);
    maximizeVelocities(points);
    computeDistance(points);
    computeCurvature(points);

    constrainCentripetalAcceleration(points);
    constrainLinearAcceleration(points);

    computeTimes(points);
    computeAccelerations(points);
    computeJolts(points);
    return new DTHolonomicTrajectory(points);
  }

  private Point[] generatePoints(DTSpline<?> spline) {
    List<Point> pointList = new ArrayList<>();
    for (int i = 0; i < spline.length(); i++) {
      Point start = createPoint(spline, i);
      pointList.add(start);

      DTVector2dR endPos = spline.getPosition(i + 1);
      recursiveSplit(pointList, spline, i, i + 1, start.position, endPos);
    }
    pointList.add(createPoint(spline, spline.length()));
    return pointList.toArray(Point[]::new);
  }

  private void recursiveSplit(List<Point> pointList, DTSpline<?> spline, double minU, double maxU,
                              DTVector2dR startPos, DTVector2dR endPos) {
    if (maxU - minU <= deltaU) {
      double distance = endPos.clone()
                              .subtract(startPos)
                              .getNorm();
      if (distance <= distanceBetweenPoints) return;
    }

    // Split in half and repeat
    double midU = 0.5 * (minU + maxU);
    Point midPoint = createPoint(spline, midU);

    recursiveSplit(pointList, spline, minU, midU, startPos, midPoint.position);
    pointList.add(midPoint);
    recursiveSplit(pointList, spline, midU, maxU, midPoint.position, endPos);
  }

  private void maximizeVelocities(Point[] points) {
    for (Point point : points) {
      point.velocity.normalize(velocityLimit.maxVelocityTranslation);
      if (point.velocity.getR() > velocityLimit.maximumAngularVelocity) {
        point.velocity.multiply(velocityLimit.maximumAngularVelocity / point.velocity.getR());
      }
      point.limitingConstraint = Constraint.VELOCITY;
    }
  }

  private void constrainCentripetalAcceleration(Point[] points) {
    for (int i = 1; i < points.length - 1; i++) {
      if (Double.isInfinite(points[i].curvature)) {
        points[i].velocity.multiply(0);
        points[i].limitingConstraint = Constraint.CENTRIPETAL;
        continue;
      }

      double maxVelocity = Math.max(Math.sqrt(accelLimit.maxTranslation / points[i].curvature),
                                    velocityLimit.minimumLinearVelocity);
      double currentVelocity = points[i].velocity.getNorm();
      if (currentVelocity > maxVelocity) {
        points[i].velocity.normalize(maxVelocity);
        points[i].limitingConstraint = Constraint.CENTRIPETAL;
      }
    }
  }

  private void constrainLinearAcceleration(Point[] points) {
    int end = points.length - 1;
    points[0].velocity.set(points[1].position.clone()
                                             .subtract(points[0].position))
                      .normalize(velocityLimit.minimumLinearVelocity);
    points[end].velocity.set(points[end].position.clone()
                                                 .subtract(points[end - 1].position))
                        .normalize(velocityLimit.minimumLinearVelocity);

    for (int i = 1, j = end - 1; i < points.length; i++, j--) {
      // Acceleration
      double prevVelocity = points[i - 1].velocity.getNorm(); // 0
      double maxAccel = computeMaxLinearAccel(points[i - 1], points[i]);
      double distance = points[i].distance - points[i - 1].distance;
      double maxVelocity =
          Math.max(velocityLimit.minimumLinearVelocity,
                   Math.sqrt(prevVelocity * prevVelocity + 2 * maxAccel * distance));

      double currentVelocity = points[i].velocity.getNorm();
      if (currentVelocity > maxVelocity) {
        points[i].velocity.multiply(maxVelocity / currentVelocity);
        points[i].limitingConstraint = Constraint.ACCELERATION;
      }

      // Deceleration
      prevVelocity = points[j + 1].velocity.getNorm();
      maxAccel = computeMaxLinearAccel(points[j + 1], points[j]);
      distance = points[j + 1].distance - points[j].distance;
      maxVelocity = Math.max(velocityLimit.minimumLinearVelocity,
                             Math.sqrt(prevVelocity * prevVelocity + 2 * maxAccel * distance));

      currentVelocity = points[j].velocity.getNorm();
      if (currentVelocity > maxVelocity) {
        points[j].velocity.multiply(maxVelocity / currentVelocity);
        points[j].limitingConstraint = Constraint.ACCELERATION;
      }
    }
  }

  private double computeMaxLinearAccel(Point start, Point end) {
    if (Double.isInfinite(start.curvature)) {
      return accelLimit.maxTranslation;
    }
    double velocity = start.velocity.getNorm();
    double centripetalAccel = velocity * velocity * (start.curvature + end.curvature) * .5;
    if (centripetalAccel >= accelLimit.maxTranslation) {
      return 0;
    }
    return Math.sqrt(accelLimit.maxTranslation * accelLimit.maxTranslation
        - centripetalAccel * centripetalAccel);
  }

  private static Point createPoint(DTSpline<?> spline, double u) {
    Point p = new Point();
    p.position = spline.getPosition(u);
    p.velocity = spline.getVelocity(u);
    p.u = u;
    return p;
  }

  private static void computeDistance(Point[] points) {
    points[0].distance = 0;

    double distance = 0;
    for (int i = 1; i < points.length; i++) {
      distance += points[i].position.clone()
                                    .subtract(points[i - 1].position)
                                    .getNorm();
      points[i].distance = distance;
    }
  }

  private static void computeCurvature(Point[] points) {
    int end = points.length - 1;
    points[0].curvature = 0;
    points[end].curvature = 0;

    for (int i = 1; i < end; i++) {
      points[i].curvature = computeCurvature(points[i - 1], points[i], points[i + 1]);
    }
  }

  private static double computeCurvature(Point before, Point current, Point after) {
    double x2 = current.position.getX() - before.position.getX();
    double y2 = current.position.getY() - before.position.getY();
    double x3 = after.position.getX() - before.position.getX();
    double y3 = after.position.getY() - before.position.getY();

    double a = Math.abs(x2 * y3 - x3 * y2 * 2);
    if (a > 1e-6) {
      // Standard case
      double x2s = x2 * x2;
      double x3s = x3 * x3;
      double y2s = y2 * y2;
      double y3s = y3 * y3;
      double s1 = (x2s + y2s) * y3 - (x3s + y3s) * y2;
      double s2 = (x2s + y2s) * x3 - (x3s + y3s) * x2;
      return a / Math.hypot(s1, s2);
    }

    // Points are collinear: either straight line (0) or cusp (+∞)
    double dTheta = Math.atan2(y2 - y3, x2 - x3) - Math.atan2(y2, x2);
    return (Math.abs(dTheta) < 1e-6 || Math.abs(dTheta - Math.PI * 2) < 1e-6)
        ? Double.POSITIVE_INFINITY : 0;
  }

  private static void computeTimes(Point[] points) {
    double time = 0;
    for (int i = 1; i < points.length - 1; i++) {
      double distance = points[i + 1].distance - points[i - 1].distance;
      double avgVelocity = (points[i + 1].velocity.getNorm() + points[i - 1].velocity.getNorm());
      time += distance / avgVelocity;
      points[i].time = time;
    }
    double distance = points[points.length - 1].distance - points[points.length - 2].distance;
    double avgVelocity = 0.5 * (points[points.length - 1].velocity.getNorm()
        + points[points.length - 2].velocity.getNorm());
    time += distance / avgVelocity;
    points[points.length - 1].time = time;
  }

  private static void computeAccelerations(Point[] points) {
    for (int i = 1; i < points.length; i++) {
      points[i].acceleration = points[i].velocity.clone()
                                                 .subtract(points[i - 1].velocity)
                                                 .divide(points[i].time - points[i - 1].time);
    }
    points[0].acceleration = points[1].acceleration.clone();
  }

  private static void computeJolts(Point[] points) {
    for (int i = 1; i < points.length; i++) {
      points[i].jolt = points[i].acceleration.clone()
                                             .subtract(points[i - 1].acceleration)
                                             .divide(points[i].time - points[i - 1].time)
                                             .getNorm();
    }
    points[0].jolt = points[1].jolt;
  }
}
