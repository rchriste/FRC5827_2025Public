// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    // AprilTag layout for the current game field.
    // PhotonVision uses this to convert camera-to-tag measurements into field coordinates.
    public static AprilTagFieldLayout aprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names. These MUST match the names configured in PhotonVision on the coprocessor.
    // If the names don't match, VisionIOPhotonVision will never connect.
    public static String camera0Name = "F-STAR";
    public static String camera1Name = "F-PORT";
    public static String camera2Name = "B-STAR";
    public static String camera3Name = "B-PORT";

    // Robot-to-camera transforms.
    // Each Transform3d describes where the camera is mounted on the robot:
    //  - X forward (+inches toward front bumper)
    //  - Y left (+inches toward left side)
    //  - Z up (+inches off the floor)
    // and its rotation (yaw/pitch/roll).
    //
    // Getting these values correct is critical: if a camera is "told" it's in the wrong spot,
    // the AprilTag pose estimates will be wrong and the robot will think it's somewhere else
    // on the field.
    //
    // (Not used by Limelight, which is configured in its web UI instead.)
    public static Transform3d robotToCamera0 =
            new Transform3d(
                    Units.inchesToMeters(11.081),
                    Units.inchesToMeters(-8.834),
                    Units.inchesToMeters(8.000),
                    new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(15)));
    public static Transform3d robotToCamera1 =
            new Transform3d(
                    Units.inchesToMeters(11.081),
                    Units.inchesToMeters(8.834),
                    Units.inchesToMeters(8.000),
                    new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(-15)));
    public static Transform3d robotToCamera2 =
            new Transform3d(
                    Units.inchesToMeters(3.103),
                    Units.inchesToMeters(-6.106),
                    Units.inchesToMeters(20.316),
                    new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(195)));
    public static Transform3d robotToCamera3 =
            new Transform3d(
                    Units.inchesToMeters(3.103),
                    Units.inchesToMeters(6.106),
                    Units.inchesToMeters(20.316),
                    new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(165)));

    // Basic filtering thresholds for deciding if a vision pose is believable.
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines for vision measurement noise.
    // These are for ~1 meter distance and 1 tag, and are scaled dynamically based on:
    //  - distance to tags
    //  - number of tags seen
    public static double linearStdDevBaseline = 0.15; // Meters
    public static double angularStdDevBaseline = 0.6; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
            new double[] {
                1.0, // Camera 0
                1.0, // Camera 1
                1.5, // Camera 2
                1.5 // Camera 3
            };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
            Double.POSITIVE_INFINITY; // No rotation data available
}
