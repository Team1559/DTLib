package org.victorrobotics.dtlib.drivetrain.trajectory;

import org.victorrobotics.dtlib.drivetrain.DTAccelerationLimit;
import org.victorrobotics.dtlib.drivetrain.DTVelocityLimit;
import org.victorrobotics.dtlib.drivetrain.trajectory.DTHolonomicTrajectory.Point;
import org.victorrobotics.dtlib.math.geometry.DTVector2DR;
import org.victorrobotics.dtlib.math.spline.DTLinearSpline;
import org.victorrobotics.dtlib.math.spline.DTSpline;

import java.util.ArrayList;
import java.util.List;

public class DTHolonomicTrajectoryGenerator {
  private static final DTAccelerationLimit DEFAULT_ACCEL_LIMIT    = new DTAccelerationLimit(25, 20 * Math.PI, 0.02);
  private static final DTVelocityLimit     DEFAULT_VELOCITY_LIMIT = new DTVelocityLimit(0.05, 5, 0.1, Math.PI * 4);

  private static final double DEFAULT_DISTANCE_BETWEEN_POINTS = 0.02;
  private static final double DEFAULT_MAX_DELTA_T             = 0.126;

  private DTAccelerationLimit accelLimit;
  private DTVelocityLimit     velocityLimit;
  private double              maxDistance;
  private double              maxDeltaU;

  public DTHolonomicTrajectoryGenerator() {
    accelLimit = DEFAULT_ACCEL_LIMIT;
    velocityLimit = DEFAULT_VELOCITY_LIMIT;
    maxDistance = DEFAULT_DISTANCE_BETWEEN_POINTS;
    maxDeltaU = DEFAULT_MAX_DELTA_T;
  }

  public void setAccelLimit(DTAccelerationLimit limit) {
    accelLimit = limit;
  }

  public void setVelocityLimit(DTVelocityLimit limit) {
    velocityLimit = limit;
  }

  public void setMaxDistance(double distanceMeters) {
    maxDistance = distanceMeters;
  }

  public DTHolonomicTrajectory generate(DTSpline<?> spline) {
    Point[] points = generatePoints(spline);
    computeDistances(points);
    maximizeVelocities(points);
    applyCentripetalConstraint(points);
    applyAccelerationConstraint(points);
    applyDecelerationConstraint(points);
    computeTimes(points);
    computeAccelerations(points);
    return new DTHolonomicTrajectory(points);
  }

  private Point[] generatePoints(DTSpline<?> spline) {
    List<Point> pointList = new ArrayList<>();
    for (int i = 0; i < spline.length(); i++) {
      Point start = createPoint(spline, i);
      Point end = createPoint(spline, i + 1);
      pointList.add(start);
      recursiveSplit(pointList, spline, i, i + 1, start.position, end.position);
    }
    pointList.add(createPoint(spline, spline.length()));
    return pointList.toArray(Point[]::new);
  }

  private void recursiveSplit(List<Point> pointList, DTSpline<?> spline, double minU, double maxU, DTVector2DR startPos,
      DTVector2DR endPos) {
    DTVector2DR displacement = endPos.clone();
    displacement.subtract(startPos);
    if (maxU - minU <= maxDeltaU && displacement.hypotenuse() <= maxDistance) {
      // Points are close enough, stop splitting
      return;
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
      point.velocity.scaleHypotenuse(velocityLimit.maxVelocityTranslation);
      if (point.velocity.getR() > velocityLimit.maximumAngularVelocity) {
        point.velocity.scaleR(velocityLimit.maximumAngularVelocity);
      }
    }
  }

  private void applyCentripetalConstraint(Point[] points) {
    points[0].curvatureRadius = Double.NaN;
    points[points.length - 1].curvatureRadius = Double.NaN;
    for (int i = 1; i < points.length - 1; i++) {
      points[i].curvatureRadius = computeRadius(points[i - 1].position, points[i].position, points[i + 1].position);
      if (Double.isNaN(points[i].curvatureRadius)) {
        points[i].velocity.scaleHypotenuse(velocityLimit.minimumLinearVelocity);
      } else if (Double.isFinite(points[i].curvatureRadius)) {
        double maxVelocity = Math.sqrt(points[i].curvatureRadius * accelLimit.maxAccelTranslation);
        points[i].velocity.scaleDownHypotenuse(maxVelocity);
      }
    }
  }

  private void applyAccelerationConstraint(Point[] points) {
    points[0].velocity.set(points[1].position.clone()
                                             .subtract(points[0].position));
    points[0].velocity.scaleHypotenuse(velocityLimit.minimumLinearVelocity);
    for (int i = 1; i < points.length; i++) {
      // sqrt(v^2+2ad)
      double prevVelT = points[i - 1].velocity.hypotenuse();
      double prevMaxAccel = computeMaxAccel(points[i - 1]);
      double maxVelT = Math.sqrt(
          prevVelT * prevVelT + 2 * prevMaxAccel * (points[i].distance - points[i - 1].distance));
      points[i].velocity.scaleDownHypotenuse(maxVelT);
    }
  }

  private void applyDecelerationConstraint(Point[] points) {
    int maxIndex = points.length - 1;
    points[maxIndex].velocity.set(points[maxIndex].position.clone()
                                                           .subtract(points[maxIndex - 1].position));
    points[maxIndex].velocity.scaleHypotenuse(velocityLimit.minimumLinearVelocity);
    for (int i = maxIndex - 1; i >= 0; i--) {
      // sqrt(v^2-2ad)
      double nextVel = points[i + 1].velocity.hypotenuse();
      double nextMaxAccel = computeMaxAccel(points[i + 1]);
      double maxVel = Math.sqrt(nextVel * nextVel + 2 * nextMaxAccel * (points[i + 1].distance - points[i].distance));
      points[i].velocity.scaleDownHypotenuse(maxVel);
    }
  }

  private double computeMaxAccel(Point p) {
    double x2 = accelLimit.maxAccelTranslation;
    double y2 = p.velocity.hypotenuse();
    y2 *= y2 / p.curvatureRadius;
    x2 *= x2;
    y2 *= y2;
    double maxAccel = Math.sqrt(x2 - y2);
    return Double.isNaN(maxAccel) ? accelLimit.maxAccelTranslation : maxAccel;
  }

  private static Point createPoint(DTSpline<?> spline, double u) {
    Point p = new Point();
    p.position = spline.getPosition(u);
    p.velocity = spline.getVelocity(u);
    p.u = u;
    return p;
  }

  private static void computeDistances(Point[] points) {
    double distance = 0;
    for (int i = 1; i < points.length; i++) {
      distance += points[i].position.clone()
                                    .subtract(points[i - 1].position)
                                    .hypotenuse();
      points[i].distance = distance;
    }
  }

  private static void computeTimes(Point[] points) {
    double time = 0;
    for (int i = 1; i < points.length; i++) {
      time += (points[i].distance - points[i - 1].distance) * 2
          / (points[i].velocity.hypotenuse() + points[i - 1].velocity.hypotenuse());
      points[i].time = time;
    }
  }

  private static void computeAccelerations(Point[] points) {
    points[0].acceleration = new DTVector2DR();
    for (int i = 1; i < points.length; i++) {
      points[i].acceleration = points[i].velocity.clone()
                                                 .subtract(points[i - 1].velocity)
                                                 .multiply(1 / (points[i].time - points[i - 1].time));
    }
  }

  private static double computeRadius(DTVector2DR before, DTVector2DR current, DTVector2DR after) {
    double x2 = current.getX() - before.getX();
    double x3 = after.getX() - before.getX();
    double y2 = current.getY() - before.getY();
    double y3 = after.getY() - before.getY();

    double a = x2 * y3 - x3 * y2;
    if (Math.abs(a) >= 1e-9) {
      // Standard case
      double x2y2 = (x2 * x2 + y2 * y2);
      double x3y3 = (x3 * x3 + y3 * y3);
      double x = x3 * x2y2 - x2 * x3y3;
      double y = y3 * x2y2 - y2 * x3y3;
      return Math.hypot(x, y) / Math.abs(2 * a);
    }

    double d1 = current.clone()
                       .subtract(before)
                       .theta();
    double d3 = current.clone()
                       .subtract(after)
                       .theta();
    if (Math.abs(d3 - d1) < 1e-3) {
      // 2 is beyond 1 or 3
      // 1----3--2
      return Double.NaN;
    } else {
      // Current is between others
      // 1---2---3
      return Double.POSITIVE_INFINITY;
    }
  }

  public static void main(String... args) {
    DTHolonomicTrajectoryGenerator generator = new DTHolonomicTrajectoryGenerator();
    DTLinearSpline spline1 = new DTLinearSpline(new DTVector2DR(), new DTVector2DR(0, 1, 0));
    spline1.appendSegment(new DTVector2DR());
    DTHolonomicTrajectory traj1 = generator.generate(spline1);
    for (Point p : traj1) {
      System.out.println(p);
    }
  }
}
