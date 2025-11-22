package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

public final class Constants {
    public static final boolean simWithVision = false;

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

    public static final Mode currentMode;

    // RoboRIO serial numbers in hex string format
    private static final String JACQUES_RIO_ID = "023AABC3";
    private static final String AGNES_RIO_ID = "030588d2"; // Replace with Agnes' RIO serial

    // Determine which robot we're on
    static {
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
