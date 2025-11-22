package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

public final class Constants {
    public static final boolean simWithVision = false;

    // CAN IDs for our hardware. Putting them here keeps them in one place so we don't "hardcode"
    // numbers throughout the project. If wiring changes, only update Constants.
    public static final int elevatorFollowMotorOpenCanbus_ID = 22;
    public static final int elevatorLeadMotorOpenCanbus_ID = 21;

    public static final int endEffectorCanbus_ID = 30;

    public static final int climbOpenCanbus_ID = 50;
    public static final int stationIntakeCan_ID = 51;
    public static final int intakeRightMotorCanbus_ID = 59;
    public static final int intakeLeftMotorCanbus_ID = 60;

    // AdvantageScope constants
    public static final Mode simMode = Mode.SIM;
    public static final boolean tuningMode = false;

    /**
     * The mode the robot code is currently running in.
     *
     * <p>AdvantageKit and our IO layering depend on knowing what hardware exists. We use this field
     * so RobotContainer can create the right IO implementations:
     *
     * <ul>
     *   <li>{@link Mode#COMPETITION}: full competition robot with all mechanisms and cameras
     *   <li>{@link Mode#AGNES}: development swerve-only robot (different hardware installed)
     *   <li>{@link Mode#SIM}: physics simulation on a computer
     *   <li>{@link Mode#REPLAY}: play back a log file to analyze past matches
     * </ul>
     */
    public static final Mode currentMode;

    // RoboRIO serial numbers in hex string format
    // Each real roboRIO has a unique serial number. We use it to automatically pick a hardware
    // config without changing code.
    private static final String JACQUES_RIO_ID = "023AABC3";
    private static final String AGNES_RIO_ID = "030588d2"; // Replace with Agnes' RIO serial

    // Determine which robot we're on
    static {
        // If we are not running on a real roboRIO, we are in simulation or log replay.
        if (!RobotBase.isReal()) {
            currentMode = simMode;
        } else {
            String serialNum = String.format("%s", RobotController.getSerialNumber());
            if (serialNum.equals(JACQUES_RIO_ID)) {
                System.out.println("Loading Jacques (Production) configuration");
                currentMode = Mode.COMPETITION;
            } else if (serialNum.equals(AGNES_RIO_ID)) {
                System.out.println("Loading Agnes (Development) configuration");
                currentMode = Mode.AGNES;
            } else {
                System.err.println("WARNING: Unknown RoboRIO ID: " + serialNum);
                System.err.println("Defaulting to Jacques configuration");
                currentMode = Mode.COMPETITION;
            }
        }
    }

    public static enum Mode {

        /** Running on the real competition robot (Jacques). */
        COMPETITION,

        /** Running on the swerve 3 bot (Agnes). */
        AGNES,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
}
