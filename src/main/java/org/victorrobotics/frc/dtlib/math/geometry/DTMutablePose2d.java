package org.victorrobotics.frc.dtlib.math.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DTMutablePose2d implements Cloneable {
    public double x;
    public double y;
    public double r;

    public DTMutablePose2d() {}

    public DTMutablePose2d(double x, double y, double r) {
        this.x = x;
        this.y = y;
        this.r = r;
    }

    public DTMutablePose2d(Pose2d pose) {
        this(pose.getX(), pose.getY(), pose.getRotation()
                                           .getDegrees());
    }

    public Pose2d toPose2d() {
        return new Pose2d(x, y, Rotation2d.fromDegrees(r));
    }

    @Override
    public DTMutablePose2d clone() {
        try {
            return (DTMutablePose2d) super.clone();
        } catch (CloneNotSupportedException e) {
            // Backup copy from the heap, should never happen
            DTMutablePose2d pose = new DTMutablePose2d();
            pose.x = x;
            pose.y = y;
            pose.r = r;
            return pose;
        }
    }
}
