package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorIOInputs {
        public boolean endEffectorConnected = false;
        public double endEffectorPositionRad = 0.0;
        public double endEffectorVelocityRadPerSec = 0.0;
        // Measured voltage, not reuested voltage
        public double endEffectorAppliedVolts = 0.0;
        // Measured current, not requested current
        public double endEffectorCurrentAmps = 0.0;
        public double motorTempCelsius = 0.0;
        public boolean pickup = false;
        public boolean dropoff = false;
        public double algaeServoRequestedPosition = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(EndEffectorIOInputs inputs) {}

    /** Run the endEffector motor at the specified voltage value. */
    public default void setEndEffectorVoltage(double voltageInVolts) {}

    /** Set the position of the servo motor to remove algae between 0.0 .. 1.0 */
    public default void setAlgaeServoPosition(double newPosition) {}
}
