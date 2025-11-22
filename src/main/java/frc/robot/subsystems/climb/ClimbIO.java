package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

    @AutoLog
    public static class ClimbIOInputs {
        public boolean winchConnected = false;
        public double winchPositionRotations = 0;
        public double winchAppliedVolts = 0;
        public double winchCurrentAmps = 0;
        public double winchTemperatureCelsius = 0;

        public boolean intakeConnected = false;
        public double intakePositionRotations = 0;
        public double intakeAppliedVolts = 0;
        public double intakeCurrentAmps = 0;
        public double intakeTemperatureCelsius = 0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClimbIOInputs inputs) {}

    /** Run the winch motor at the specified voltage value. */
    public default void setWinchVoltage(double voltage) {}

    /** Run the station intake motor at the specified voltage value. */
    public default void setIntakeVoltage(double voltage) {}
}
