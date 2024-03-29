package org.victorrobotics.dtlib.math.trajectory;

import org.victorrobotics.dtlib.math.geometry.Vector2D_R;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.NoSuchElementException;

import edu.wpi.first.math.trajectory.Trajectory;

public class HolonomicTrajectory implements Iterable<HolonomicTrajectory.Point> {
  public enum Constraint {
    VELOCITY,
    CENTRIPETAL,
    ACCELERATION,
    JOLT;
  }

  @SuppressWarnings("java:S1104") // public non-final members
  public static class Point {
    public Vector2D_R position;
    public Vector2D_R velocity;
    public Vector2D_R acceleration;
    public double     jolt;

    public double distance;
    public double time;
    public double u;
    public double curvature;

    public Constraint limitingConstraint;

    @Override
    public String toString() {
      return String.format("Point[t=%.2f d=%.3f x=%.3f y=%.3f r=%.1f v=%.3f a=%.3f j=%.0f c=%.0f %s]",
                           time, distance, position.getX(), position.getY(), position.getR(),
                           velocity.getNorm(), acceleration.getNorm(), jolt, curvature,
                           limitingConstraint);
    }
  }

  public final Point[] points;
  public final double  time;
  public final double  distance;

  public HolonomicTrajectory(Point... points) {
    this.points = points;
    time = points[points.length - 1].time;
    distance = points[points.length - 1].distance;
  }

  public Trajectory toTrajectory() {
    List<Trajectory.State> states = new ArrayList<>();
    for (int i = 0; i < points.length; i++) {
      Point p = points[i];
      states.add(new Trajectory.State(p.time, p.velocity.getNorm(), p.acceleration.getNorm(),
                                      p.position.toPose2d(), 1 / p.curvature));
    }
    return new Trajectory(states);
  }

  @Override
  public String toString() {
    StringBuilder builder = new StringBuilder("HolonomicTrajectory([\n");
    int maxNumLen = (int) Math.ceil(Math.log10(points.length - 1));
    String indexFormat = "%" + maxNumLen + "d";
    for (int i = 0; i < points.length; i++) {
      builder.append(String.format(indexFormat, i))
             .append(": ")
             .append(points[i])
             .append('\n');
    }
    return builder.append(String.format("], Time: %.1fs, Distance: %.1fm)", time, distance))
                  .toString();
  }

  @Override
  public Iterator<Point> iterator() {
    return new Iterator<>() {
      int index;

      @Override
      public boolean hasNext() {
        return index < points.length;
      }

      @Override
      public Point next() {
        if (index >= points.length) {
          throw new NoSuchElementException();
        }
        return points[index++];
      }
    };
  }
}
