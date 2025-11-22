package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.util.Units;

import frc.robot.FieldConstants;
import frc.robot.util.GeomUtil.FieldElement;
import frc.robot.util.GeomUtil.Line;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

// Odometry that clamps poses inside of field
public class SmartOdometry<T> extends Odometry<T> {
    private LoggedNetworkBoolean enableVirtualWallCollision =
            new LoggedNetworkBoolean("/SmartDashboard/Drive/EnableVirtualWallCollision", false);

    private static final Transform2d corner1Transform =
            new Transform2d(
                    Units.inchesToMeters(14.25), Units.inchesToMeters(14.5), Rotation2d.kZero);
    private static final Transform2d corner2Transform =
            new Transform2d(
                    Units.inchesToMeters(14.25), Units.inchesToMeters(-14.5), Rotation2d.kZero);
    private static final Transform2d corner3Transform =
            new Transform2d(
                    Units.inchesToMeters(-14.5), Units.inchesToMeters(-14.5), Rotation2d.kZero);
    private static final Transform2d corner4Transform =
            new Transform2d(
                    Units.inchesToMeters(-14.5), Units.inchesToMeters(14.5), Rotation2d.kZero);

    private final Line[] robotLines;
    private final FieldElement robotFieldElement;

    public SmartOdometry(
            Kinematics<?, T> kinematics,
            Rotation2d gyroAngle,
            T wheelPositions,
            Pose2d initialPoseMeters) {
        super(kinematics, gyroAngle, wheelPositions, initialPoseMeters);
        robotLines =
                GeomUtil.createLinesFromPoints(
                        initialPoseMeters.transformBy(corner1Transform).getTranslation(),
                        initialPoseMeters.transformBy(corner2Transform).getTranslation(),
                        initialPoseMeters.transformBy(corner3Transform).getTranslation(),
                        initialPoseMeters.transformBy(corner4Transform).getTranslation());
        robotFieldElement = new FieldElement(robotLines, false, initialPoseMeters.getTranslation());
    }

    private Pose2d clampPoseToField(Pose2d p) {
        FieldElement robotFieldElement = getRobotFieldElement(p);

        for (int i = 0; i < 100; i++) {
            for (FieldElement boundary : FieldConstants.allBoundaries) {
                if (boundary.intersects(robotFieldElement)) {
                    boundary.moveRobot(robotFieldElement);
                }
            }
        }
        return new Pose2d(
                new Translation2d(robotFieldElement.centerX, robotFieldElement.centerY),
                p.getRotation());
    }

    private FieldElement getRobotFieldElement(Pose2d p) {
        Translation2d corner1 = p.transformBy(corner1Transform).getTranslation();
        Translation2d corner2 = p.transformBy(corner2Transform).getTranslation();
        Translation2d corner3 = p.transformBy(corner3Transform).getTranslation();
        Translation2d corner4 = p.transformBy(corner4Transform).getTranslation();
        robotLines[0].p1x = robotLines[3].p2x = corner1.getX();
        robotLines[0].p1y = robotLines[3].p2y = corner1.getY();
        robotLines[1].p1x = robotLines[0].p2x = corner2.getX();
        robotLines[1].p1y = robotLines[0].p2y = corner2.getY();
        robotLines[2].p1x = robotLines[1].p2x = corner3.getX();
        robotLines[2].p1y = robotLines[1].p2y = corner3.getY();
        robotLines[3].p1x = robotLines[2].p2x = corner4.getX();
        robotLines[3].p1y = robotLines[2].p2y = corner4.getY();
        robotFieldElement.centerX = p.getTranslation().getX();
        robotFieldElement.centerY = p.getTranslation().getY();
        return robotFieldElement;
    }

    @Override
    public Pose2d update(Rotation2d gyroAngle, T wheelPositions) {
        Pose2d pose = super.update(gyroAngle, wheelPositions);
        if (enableVirtualWallCollision.get()) {
            pose = clampPoseToField(pose);
            resetPose(pose);
        }
        return pose;
    }
}
