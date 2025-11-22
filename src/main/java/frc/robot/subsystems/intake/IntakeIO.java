package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean hasCoral = false;

        public boolean leftConnected = false;
        public double leftMotorVelocityRadPerSec = 0;
        public double leftMotorAppliedVolts = 0; // Measured
        public double leftMotorAmps = 0; // Measured
        public double leftMotorTempCelsius = 0;

        public boolean rightConnected = false;
        public double rightMotorVelocityRadPerSec = 0;
        public double rightMotorAppliedVolts = 0; // Measured
        public double rightMotorAmps = 0; // Measured
        public double rightMotorTempCelsius = 0;

        public boolean pivotMotorConnected = false;
        public double pivotPositionRadians = 0.0;
        public double pivotVelocityRadPerSec = 0.0;
        // Measured voltage, not reuested voltage
        public double pivotMotorVoltageVolts = 0.0;
        // Measured current, not requested current
        public double pivotMotorCurrentAmps = 0.0;
        public double pivotMotorTempCelsius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {}

    /** Requests the pivot to move to a target position (in rotations) */
    public default void setPivotPosition(double targetPosition) {}

    public default void setPivotState(
            double targetPosition, double targetVelocity, double feedforwardValue) {}

    public default void setPivotVoltage(double voltage) {}

    public default void setPivotBrakeMode(boolean brake) {}

    /** Run the intake motor at the specified voltage value. */
    public default void setRollerVoltage(double leftVoltageVolts, double rightVoltageVolts) {}

    public default void updatePID(double kP, double kD) {}
}
