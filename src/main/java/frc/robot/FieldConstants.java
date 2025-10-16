package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import frc.robot.util.GeomUtil;
import frc.robot.util.GeomUtil.FieldElement;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

public class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(690.876);
    public static final double fieldWidth = Units.inchesToMeters(317);
    public static final double startingLineX =
            Units.inchesToMeters(299.438); // Measured from the inside of starting line
    public static final double algaeDiameter = Units.inchesToMeters(16);

    public static class Processor {
        public static final Pose2d centerFace =
                new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
    }

    public static class Barge {
        public static final Translation2d farCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
        public static final Translation2d middleCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
        public static final Translation2d closeCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

        // Measured from floor to bottom of cage
        public static final double deepHeight = Units.inchesToMeters(3.125);
        public static final double shallowHeight = Units.inchesToMeters(30.125);
    }

    public static class CoralStation {
        public static final double stationLength = Units.inchesToMeters(79.750);
        public static final Pose2d leftCenterFace =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(291.176),
                        Rotation2d.fromDegrees(90 - 144.011));
        public static final Pose2d rightCenterFace =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(25.824),
                        Rotation2d.fromDegrees(144.011 - 90));
    }

    public static class Reef {
        public static final double faceLength = Units.inchesToMeters(36.792600);
        public static final Translation2d center =
                new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
        public static final double faceToZoneLine =
                Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

        public static final Pose2d[] centerFaces =
                new Pose2d[6]; // Starting facing the driver station in counter-clockwise order

        public static final List<Map<ReefLevel, Pose3d>> leftBranchPositions = new ArrayList<>();
        public static final List<Map<ReefLevel, Pose3d>> rightBranchPositions = new ArrayList<>();
        public static final List<Pose2d> leftBranchPositions2d = new ArrayList<>();
        public static final List<Pose2d> rightBranchPositions2d = new ArrayList<>();

        // counter-clockwise

        public static Translation2d[][] zones = new Translation2d[6][4];
        public static double centerToFace = 0;

        static {
            var aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

            centerFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d();
            centerFaces[5] = aprilTagLayout.getTagPose(19).get().toPose2d();
            centerFaces[4] = aprilTagLayout.getTagPose(20).get().toPose2d();
            centerFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
            centerFaces[2] = aprilTagLayout.getTagPose(22).get().toPose2d();
            centerFaces[1] = aprilTagLayout.getTagPose(17).get().toPose2d();

            centerToFace = centerFaces[0].getTranslation().getDistance(center);

            // Initialize branch positions
            for (int face = 0; face < 6; face++) {
                Map<ReefLevel, Pose3d> fillRight = new EnumMap<>(ReefLevel.class);
                Map<ReefLevel, Pose3d> fillLeft = new EnumMap<>(ReefLevel.class);
                for (var level : ReefLevel.values()) {
                    Pose2d poseDirection =
                            new Pose2d(center, Rotation2d.fromDegrees(180 + (60 * face)));
                    double adjustX = Units.inchesToMeters(30.738);
                    double adjustY = Units.inchesToMeters(6.469);

                    fillRight.put(
                            level,
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    adjustY,
                                                                    new Rotation2d()))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    adjustY,
                                                                    new Rotation2d()))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(level.pitch),
                                            poseDirection.getRotation().getRadians())));
                    fillLeft.put(
                            level,
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    -adjustY,
                                                                    new Rotation2d()))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    -adjustY,
                                                                    new Rotation2d()))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(level.pitch),
                                            poseDirection.getRotation().getRadians())));
                }
                rightBranchPositions.add(fillRight);
                rightBranchPositions2d.add(fillRight.get(ReefLevel.L1).toPose2d());
                leftBranchPositions.add(fillLeft);
                leftBranchPositions2d.add(fillLeft.get(ReefLevel.L1).toPose2d());

                double zoneX = Units.feetToMeters(10);

                Transform2d[] transforms = {
                    new Transform2d(
                            zoneX,
                            (zoneX + centerToFace) * Math.tan(Units.degreesToRadians(30)),
                            new Rotation2d(0.0)),
                    new Transform2d(
                            zoneX,
                            -(zoneX + centerToFace) * Math.tan(Units.degreesToRadians(30)),
                            new Rotation2d(0))
                };

                for (int i = 0; i < centerFaces.length; i++) {
                    zones[i][0] = center;
                    for (int j = 0; j < transforms.length; j++) {
                        zones[i][j + 1] =
                                centerFaces[i].transformBy(transforms[j]).getTranslation();
                    }
                    zones[i][3] = center;
                }
            }
        }

        public static ReefLevel algaeLevelFromZone(int zone) {
            return zone % 2 == 0 ? ReefLevel.L3 : ReefLevel.L2;
        }
    }

    public enum ReefLevel {
        L1(Units.inchesToMeters(25.0), 0),
        L2(Units.inchesToMeters(31.875 + 1.8), -35),
        L3(Units.inchesToMeters(47.625 + 1.8), -35),
        L4(Units.inchesToMeters(72 + 1.0), -90);

        ReefLevel(double height, double pitch) {
            this.height = height;
            this.pitch = pitch; // in degrees
        }

        public static ReefLevel fromLevel(int level) {
            return Arrays.stream(values())
                    .filter(height -> height.ordinal() == level)
                    .findFirst()
                    .orElse(L4);
        }

        public double getHeightInMeters() {
            return height;
        }

        public final double height;
        public final double pitch;
    }

    private static final Translation2d[] blueReefPoints =
            new Translation2d[] {
                new Translation2d(4.488, 5.0),
                new Translation2d(5.34, 4.52),
                new Translation2d(5.34, 3.53),
                new Translation2d(4.488, 3.05),
                new Translation2d(3.64, 3.53),
                new Translation2d(3.64, 4.52)
            };
    public static final FieldElement blueReefBoundary =
            new FieldElement(GeomUtil.createLinesFromPoints(blueReefPoints), true, Reef.center);
    public static final FieldElement redReefBoundary =
            new FieldElement(
                    GeomUtil.createLinesFromPoints(
                            Arrays.stream(blueReefPoints)
                                    .map(
                                            translation ->
                                                    new Translation2d(
                                                            fieldLength - translation.getX(),
                                                            fieldWidth - translation.getY()))
                                    .toArray(Translation2d[]::new)),
                    true,
                    new Translation2d(
                            fieldLength - Reef.center.getX(), fieldWidth - Reef.center.getY()));

    // CCW because we need to stay inside
    public static final FieldElement gameFieldBoundary =
            new FieldElement(
                    GeomUtil.createLinesFromPoints(
                            new Translation2d[] {
                                new Translation2d(0, 1.23),
                                new Translation2d(0, 6.8),
                                new Translation2d(1.73, 8.05),
                                new Translation2d(15.83, 8.05),
                                new Translation2d(17.55, 6.79),
                                new Translation2d(17.55, 1.23),
                                new Translation2d(15.85, 0),
                                new Translation2d(1.69, 0)
                            }),
                    false,
                    new Translation2d(fieldLength / 2, fieldWidth / 2));

    public static final FieldElement[] allBoundaries = {
        gameFieldBoundary, blueReefBoundary, redReefBoundary
    };
}
