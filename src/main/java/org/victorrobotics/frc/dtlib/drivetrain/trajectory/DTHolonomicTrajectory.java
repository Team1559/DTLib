package org.victorrobotics.frc.dtlib.drivetrain.trajectory;

import org.victorrobotics.frc.dtlib.math.geometry.DTVector2DR;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.trajectory.Trajectory;

public class DTHolonomicTrajectory implements Iterable<DTHolonomicTrajectory.Point> {
    public static class Point implements Interpolatable<Point> {
        public DTVector2DR position;
        public DTVector2DR velocity;
        public DTVector2DR acceleration;
        public double      time;
        public double      u;
        public double      distance;
        public double      curvatureRadius;

        @Override
        public Point interpolate(Point endValue, double t) {
            Point ret = new Point();

            ret.position = position.interpolate(endValue.position, t);
            ret.velocity = velocity.interpolate(endValue.velocity, t);
            ret.acceleration = acceleration.interpolate(endValue.acceleration, t);

            ret.time = MathUtil.interpolate(time, endValue.time, t);
            ret.distance = MathUtil.interpolate(distance, endValue.distance, t);
            ret.curvatureRadius = MathUtil.interpolate(curvatureRadius, endValue.curvatureRadius, t);

            return ret;
        }

        @Override
        public String toString() {
            return String.format("Point[t=%.2f Pos=%s Vel=%s Acc=%s c=%.1f]", time, position, velocity, acceleration,
                    curvatureRadius);
        }
    }

    public final Point[] points;
    public final double  time;

    public DTHolonomicTrajectory(Point... points) {
        this.points = points;
        time = points[points.length - 1].time;
    }

    public Trajectory toTrajectory() {
        List<Trajectory.State> states = new ArrayList<>();
        for (int i = 0; i < points.length; i++) {
            Point p = points[i];
            states.add(new Trajectory.State(p.time, p.velocity.hypotenuse(), p.acceleration.hypotenuse(),
                    p.position.toPose2d(), 1 / p.curvatureRadius));
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

    @Override
    public Iterator<Point> iterator() {
        return Arrays.asList(points)
                     .iterator();
    }
}
