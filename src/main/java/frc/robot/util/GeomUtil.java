// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Geometry utilities for working with translations, rotations, transforms, and poses. */
public class GeomUtil {
    /**
     * Creates a pure translating transform
     *
     * @param translation The translation to create the transform with
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Translation2d translation) {
        return new Transform2d(translation, new Rotation2d());
    }

    /**
     * Creates a pure translating transform
     *
     * @param x The x coordinate of the translation
     * @param y The y coordinate of the translation
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(double x, double y) {
        return new Transform2d(x, y, new Rotation2d());
    }

    /**
     * Creates a pure rotating transform
     *
     * @param rotation The rotation to create the transform with
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Rotation2d rotation) {
        return new Transform2d(new Translation2d(), rotation);
    }

    /**
     * Converts a Pose2d to a Transform2d to be used in a kinematic chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }

    public static Pose2d inverse(Pose2d pose) {
        Rotation2d rotationInverse = pose.getRotation().unaryMinus();
        return new Pose2d(
                pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
    }

    /**
     * Converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic
     * chain
     *
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    public static Pose2d toPose2d(Transform2d transform) {
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * Creates a pure translated pose
     *
     * @param translation The translation to create the pose with
     * @return The resulting pose
     */
    public static Pose2d toPose2d(Translation2d translation) {
        return new Pose2d(translation, new Rotation2d());
    }

    /**
     * Creates a pure rotated pose
     *
     * @param rotation The rotation to create the pose with
     * @return The resulting pose
     */
    public static Pose2d toPose2d(Rotation2d rotation) {
        return new Pose2d(new Translation2d(), rotation);
    }

    /**
     * Multiplies a twist by a scaling factor
     *
     * @param twist The twist to multiply
     * @param factor The scaling factor for the twist components
     * @return The new twist
     */
    public static Twist2d multiply(Twist2d twist, double factor) {
        return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
    }

    /**
     * Converts a Pose3d to a Transform3d to be used in a kinematic chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    public static Transform3d toTransform3d(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    /**
     * Converts a Transform3d to a Pose3d to be used as a position or as the start of a kinematic
     * chain
     *
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    public static Pose3d toPose3d(Transform3d transform) {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * Converts a ChassisSpeeds to a Twist2d by extracting two dimensions (Y and Z). chain
     *
     * @param speeds The original translation
     * @return The resulting translation
     */
    public static Twist2d toTwist2d(ChassisSpeeds speeds) {
        return new Twist2d(
                speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    /**
     * Creates a new pose from an existing one using a different translation value.
     *
     * @param pose The original pose
     * @param translation The new translation to use
     * @return The new pose with the new translation and original rotation
     */
    public static Pose2d withTranslation(Pose2d pose, Translation2d translation) {
        return new Pose2d(translation, pose.getRotation());
    }

    /**
     * Creates a new pose from an existing one using a different rotation value.
     *
     * @param pose The original pose
     * @param rotation The new rotation to use
     * @return The new pose with the original translation and new rotation
     */
    public static Pose2d withRotation(Pose2d pose, Rotation2d rotation) {
        return new Pose2d(pose.getTranslation(), rotation);
    }

    public static Translation2d findCentroid(Translation2d[] zone) {
        // check endpoints to see if they are the same
        double averageX = 0;
        double averageY = 0;
        for (int i = 0; i < zone.length - 1; i++) {
            averageX += zone[i].getX();
            averageY += zone[i].getY();
        }
        averageX /= zone.length - 1;
        averageY /= zone.length - 1;
        return new Translation2d(averageX, averageY);
    }

    /**
     * @param a is the first point of the line
     * @param b is the second point of the line
     * @param c is the point you want to check
     * @return 1 if c is above the line, -1 if c is below the line, 0 if c is on the line
     */
    public static int checkPointToLine(Translation2d a, Translation2d b, Translation2d c) {
        if (a.getX() == b.getX()) {
            // Checks if line is vertical (slope would be undefined)
            if (c.getX() > a.getX()) return 1;
            else if (c.getX() < a.getX()) return -1;
            else return 0;
        } else {
            double slope = (b.getY() - a.getY()) / (b.getX() - a.getX());
            double yIntercept = a.getY() - slope * a.getX();
            double lineAtCX = slope * c.getX() + yIntercept;
            if (lineAtCX > c.getY()) return 1;
            else if (lineAtCX < c.getY()) return -1;
            else return 0;
        }
    }

    public static boolean inZone(Pose2d robotPose, Translation2d[] zone) {
        Translation2d centroid = findCentroid(zone);
        for (int i = 0; i < zone.length - 1; i++) {
            Translation2d a = zone[i];
            Translation2d b = zone[i + 1];
            if (checkPointToLine(a, b, centroid)
                    != checkPointToLine(a, b, robotPose.getTranslation())) return false;
        }
        return true;
    }

    /**
     * Returns the index of the first zone that matches the checks Returns -1 if the robot is not in
     * a zone
     *
     * @param robotPose
     * @param zones
     * @return
     */
    public static int getCurrentZone(Pose2d robotPose, Translation2d[][] zones) {
        for (int i = 0; i < zones.length; i++) {
            if (inZone(robotPose, zones[i])) return i;
        }
        return -1;
    }

    public static class Line {
        public double p1x, p1y;
        public double p2x, p2y;
        public double normal;

        public Line(Translation2d pose1, Translation2d pose2) {
            this.p1x = pose1.getX();
            this.p1y = pose1.getY();
            this.p2x = pose2.getX();
            this.p2y = pose2.getY();
        }

        private int orientation(double px, double py, double qx, double qy, double rx, double ry) {
            double val = (qy - py) * (rx - qx) - (qx - px) * (ry - qy);
            if (val == 0) return 0; // collinear
            return (val > 0) ? 1 : 2; // clockwise or counterclockwise
        }

        private boolean onSegment(
                double px, double py, double qx, double qy, double rx, double ry) {
            if (qx <= Math.max(px, rx)
                    && qx >= Math.min(px, rx)
                    && qy <= Math.max(py, ry)
                    && qy >= Math.min(py, ry)) {
                return true;
            }
            return false;
        }

        public boolean intersects(Line other) {
            double o1 = orientation(this.p1x, this.p1y, this.p2x, this.p2y, other.p1x, other.p1y);
            double o2 = orientation(this.p1x, this.p1y, this.p2x, this.p2y, other.p2x, other.p2y);
            double o3 = orientation(other.p1x, other.p1y, other.p2x, other.p2y, this.p1x, this.p1y);
            double o4 = orientation(other.p1x, other.p1y, other.p2x, other.p2y, this.p2x, this.p2y);

            // General case
            if (o1 != o2 && o3 != o4) {
                return true;
            }

            // Special cases
            if (o1 == 0 && onSegment(this.p1x, this.p1y, other.p1x, other.p1y, this.p2x, this.p2y))
                return true;
            if (o2 == 0 && onSegment(this.p1x, this.p1y, other.p2x, other.p2y, this.p2x, this.p2y))
                return true;
            if (o3 == 0
                    && onSegment(other.p1x, other.p1y, this.p1x, this.p1y, other.p2x, other.p2y))
                return true;
            if (o4 == 0
                    && onSegment(other.p1x, other.p1y, this.p2x, this.p2y, other.p2x, other.p2y))
                return true;

            return false; // No intersection
        }

        public void translate(double x, double y) {
            p1x += x;
            p1y += y;
            p2x += x;
            p2y += y;
        }
    }

    public static class FieldElement {
        private static final double boundaryAccuracy = 0.01;

        private Line[] lines;
        public boolean pushOutward;
        public double centerX, centerY;

        public FieldElement(Line[] lines, boolean pushOutward, Translation2d center) {
            this.lines = lines;
            this.centerX = center.getX();
            this.centerY = center.getY();
            this.pushOutward = pushOutward;
            calculateNormals();
        }

        // only works if inside of area or intersecting the edge of the area. THe method of moving
        // is not quite optimal but it will work for now
        // the best method would to move it in the purpendicual direction of the intersection.
        public boolean intersects(FieldElement other) {
            for (Line line : this.lines) {
                for (Line otherLine : other.lines) {
                    if (line.intersects(otherLine)) return true;
                }
            }
            return false;
        }

        public void moveRobot(FieldElement robot) {
            for (Line line : this.lines) {
                boolean intersects = false;
                for (Line otherLine : robot.lines) {
                    if (line.intersects(otherLine)) {
                        intersects = true;
                        break;
                    }
                }
                if (intersects) {
                    if (!pushOutward) {
                        robot.translate(
                                -boundaryAccuracy * Math.cos(line.normal),
                                -boundaryAccuracy * Math.sin(line.normal));
                    } else {
                        robot.translate(
                                boundaryAccuracy * Math.cos(line.normal),
                                boundaryAccuracy * Math.sin(line.normal));
                    }
                }
            }
        }

        public void translate(double x, double y) {
            for (Line line : lines) {
                line.translate(x, y);
            }
            centerX += x;
            centerY += y;
        }

        private void calculateNormals() {
            for (Line line : lines) {
                line.normal = Math.atan2(line.p2y - line.p1y, line.p2x - line.p1x) + Math.PI / 2;
                double centerOfLineX = (line.p1x + line.p2x) / 2;
                double centerOfLineY = (line.p1y + line.p2y) / 2;
                double angleToCenter = Math.atan2(centerY - centerOfLineY, centerX - centerOfLineX);
                if (Math.abs(angleToCenter - line.normal) > 90) {
                    line.normal += Math.PI;
                }
            }
        }
    }

    public static Line[] createLinesFromPoints(Translation2d... points) {
        Line[] lines = new Line[points.length];
        for (int i = 0; i < points.length - 1; i++) {
            lines[i] = new Line(points[i], points[i + 1]);
        }
        lines[points.length - 1] = new Line(points[points.length - 1], points[0]);
        return lines;
    }
}
