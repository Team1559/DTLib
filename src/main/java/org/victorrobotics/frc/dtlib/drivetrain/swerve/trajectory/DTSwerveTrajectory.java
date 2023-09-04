package org.victorrobotics.frc.dtlib.drivetrain.swerve.trajectory;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class DTSwerveTrajectory {
  public static class Point {
    public Pose2d pose;
    public double time;
    public double distance;
    public double commandVelocity;
    public double predictVelocity;
    public double acceleration;
    public double curvature;

    public Point() {}

    public Point(Pose2d pose, double time, double distance, double commandVelocity,
        double predictVelocity, double acceleration, double curvature) {
      this.pose = pose;
      this.time = time;
      this.distance = distance;
      this.commandVelocity = commandVelocity;
      this.predictVelocity = predictVelocity;
      this.acceleration = acceleration;
      this.curvature = curvature;
    }

    @Override
    public String toString() {
      return String.format("Point(X: %.2f, Y: %.2f, R: %.0f, C: %.0f, CV: %.1f, PV: %.1f, D: %.1f)",
          pose.getX(), pose.getY(), pose.getRotation()
                                        .getDegrees(),
          curvature, commandVelocity, predictVelocity, distance);
    }
  }

  public final Point[] points;
  public final int     length;
  public final double  time;

  public DTSwerveTrajectory(Point... points) {
    this.points = points;
    length = points.length;
    time = points[length - 1].time;
  }

  public Trajectory toTrajectory() {
    List<Trajectory.State> states = new ArrayList<>();
    for (int i = 0; i < points.length; i++) {
      Point p = points[i];
      states.add(
          new Trajectory.State(p.time, p.predictVelocity, p.acceleration, p.pose, p.curvature));
    }
    return new Trajectory(states);
  }

  @Override
  public String toString() {
    return toString("");
  }

  public String toString(String delimiter) {
    StringBuilder builder = new StringBuilder();
    builder.append("DTSwerveTrajectory([")
           .append(delimiter);
    int maxNumLen = (int) Math.ceil(Math.log10(points.length - 1));
    String formatString = String.format("%%%dd: ", maxNumLen);
    for (int i = 0; i < points.length; i++) {
      builder.append(String.format(formatString, i))
             .append(points[i])
             .append(delimiter);
    }
    builder.append(String.format("], T: %.1f)", time));
    return builder.toString();
  }
}
