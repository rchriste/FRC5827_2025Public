package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {

        public boolean elevatorConnected = false;
        public boolean coralSecure = false;

        public double positionMeters = 0.0;
        public double velocityMetersPerSecond = 0.0;

        public double leadMotorAppliedVolts = 0.0; // Measured
        public double followMotorAppliedVolts = 0.0; // Measured

        public double leadMotorCurrentAmps = 0.0; // Measured
        public double followMotorCurrentAmps = 0.0; // Measured

        public double leadMotorTemperature = 0.0;
        public double followMotorTemperature = 0.0;

        // TODO: If we have a limit switch, add it here
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /** Sets the elevator voltage to desired value. */
    public default void setElevatorVoltage(double voltage) {}

    /** Sets motor position to desired value in meters. */
    public default void setElevatorPosition(double targetPositionMeters) {}

    /** Setting position with velocity and FF for motion profile. */
    public default void setElevatorState(
            double targetPositionMeters, double targetVelocityMpS, double feedForwardVoltage) {}

    public default void resetPosition() {}

    /** No kI value because we don't use it */
    public default void updatePID(double kP, double kD) {}
}
