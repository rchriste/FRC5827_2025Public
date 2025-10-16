package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
    private final EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs;

    private final LoggedTunableNumber intakeVoltage =
            new LoggedTunableNumber("EndEffector/IntakeVoltage", 8.0);
    private final LoggedTunableNumber slowIntakeVoltage =
            new LoggedTunableNumber("EndEffector/SlowIntakeVoltage", 4.5);
    private final LoggedTunableNumber ejectVoltage =
            new LoggedTunableNumber("EndEffector/EjectVoltage", 12.0);
    private final LoggedTunableNumber ejectDelay =
            new LoggedTunableNumber("EndEffector/EjectDelay", 0.5); // measured in seconds
    private final LoggedTunableNumber algaeServoOpenPosition =
            new LoggedTunableNumber("EndEffector/AlgaeServoOpenPosition", 1.0);
    private final LoggedTunableNumber algaeServoClosedPosition =
            new LoggedTunableNumber("EndEffector/AlgaeServoClosedPosition", 0.3);

    public EndEffector(EndEffectorIO io) {
        this.io = io;
        this.inputs = new EndEffectorIOInputsAutoLogged();
        io.setAlgaeServoPosition(algaeServoClosedPosition.get());
    }

    public Command takeFromIntake() {
        return this.startEnd(() -> this.startTakingFromIntake(), () -> this.stop())
                .until(() -> inputs.dropoff)
                .andThen(
                        this.startEnd(() -> this.slowTakingFromIntake(), () -> this.stop())
                                .until(() -> !inputs.pickup));
    }

    public Command takeFromIntakeFast() {
        return this.startEnd(() -> this.startTakingFromIntake(), () -> this.stop())
                .until(() -> inputs.dropoff && !inputs.pickup);
    }

    public Command openLoopEnableMotor() {
        return this.startEnd(() -> this.startTakingFromIntake(), () -> this.stop());
    }

    public Command ejectToReef() {
        // keeps the motor running after the sensor for a while to clear (500 ms)
        return this.startEnd(() -> this.startEjectingToReef(), () -> {})
                .until(() -> !inputs.pickup && !inputs.dropoff)
                .andThen(Commands.waitSeconds(ejectDelay.getAsDouble()))
                .finallyDo(() -> this.stop());
        // return this.startEnd(() -> this.startEjectingToReef(), () -> this.stop());
    }

    public Command reverse() {
        return this.startEnd(
                () -> io.setEndEffectorVoltage(-intakeVoltage.get()),
                () -> io.setEndEffectorVoltage(0));
    }

    public Command algaeRemoverOpen() {
        return this.runOnce(() -> this.io.setAlgaeServoPosition(this.algaeServoOpenPosition.get()));
    }

    public Command algaeRemoverClose() {
        return this.runOnce(
                () -> this.io.setAlgaeServoPosition(this.algaeServoClosedPosition.get()));
    }

    private void startTakingFromIntake() {
        io.setEndEffectorVoltage(intakeVoltage.getAsDouble());
    }

    private void slowTakingFromIntake() {
        io.setEndEffectorVoltage(slowIntakeVoltage.getAsDouble());
    }

    public Command drawback() {
        return this.startEnd(
                        () -> io.setEndEffectorVoltage(-ejectVoltage.getAsDouble()),
                        () -> this.stop())
                .until(() -> !inputs.dropoff)
                .withTimeout(0.2);
    }

    public Command shortDrawback() {
        return this.startEnd(
                        () -> io.setEndEffectorVoltage(-slowIntakeVoltage.getAsDouble()),
                        () -> this.stop())
                .withTimeout(0.07);
    }

    public void startEjectingToReef() {
        io.setEndEffectorVoltage(ejectVoltage.getAsDouble());
    }

    public boolean hasCoral() {
        return inputs.pickup || inputs.dropoff;
    }

    private void stop() {
        io.setEndEffectorVoltage(0.0);
    }

    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();
        io.updateInputs(inputs);
        Logger.processInputs("endEffector", inputs);
        Logger.recordOutput(
                "PeriodicTime/EndEffector", (Timer.getFPGATimestamp() - startTime) * 1000);
    }

    /** Check if coral in the end effector will cause the elevator to jam when it is moved. */
    public boolean isCoralBlockingElevator() {
        return inputs.pickup;
    }
}
