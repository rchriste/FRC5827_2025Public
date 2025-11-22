package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.FieldConstants.ReefLevel;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs;
    private final LoggedTunableNumber openLoopVoltage =
            new LoggedTunableNumber("Elevator/Voltage", 1.0);
    private final ElevatorFeedforward feedForward;
    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State currentState, goalState;
    private final BooleanSupplier elevatorBlocked;

    static final double bottomPositionMeters = 0;

    static final double SPROCKET_RADIUS_METERS = Units.inchesToMeters(1.9153 / 2.0);
    static final double GEAR_REDUCTION = 5.0;

    static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 8);
    static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0);

    static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.3);
    static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.2);
    static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 2.6);
    static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.1);

    static final LoggedTunableNumber trapezoidMaxVelocity =
            new LoggedTunableNumber("Elevator/Velocity", 3.0);
    static final LoggedTunableNumber trapezoidMaxAcceleration =
            new LoggedTunableNumber("Elevator/Acceleration", 6.0);
    static final LoggedTunableNumber zeroingVoltage =
            new LoggedTunableNumber("Elevator/ZeroingVoltage", 0.5);

    static final LoggedTunableNumber elevatorOffsetMeters =
            new LoggedTunableNumber("Elevator/Offset", 0.15);
    static final LoggedTunableNumber algaeOffsetMeters =
            new LoggedTunableNumber("Elevator/AlgaeOffset", 0.45);
    static final LoggedTunableNumber algaeBumpOffsetMeters =
            new LoggedTunableNumber("Elevator/AlgaeBumpOffset", 0.37);
    static final LoggedTunableNumber stationIntakeHeight =
            new LoggedTunableNumber("Elevator/StationIntakeHeight", 0.26);

    static final double positionToleranceMeters =
            Units.rotationsToRadians(convertMetersToRotations(0.2)); // 20 cm
    private final double velocityToleranceMeters =
            Units.rotationsToRadians(convertMetersToRotations(0.2));
    private final Debouncer zeroedDebouncer = new Debouncer(0.2);
    private boolean atSetpoint = false;
    private boolean doProfiling = false;
    private boolean zeroed = false;

    private final Alert coralBlockingAlert =
            new Alert("Coral is blocking the elevator. Cannot move.", AlertType.kWarning);

    /**
     * Utlity method to convert meters to rotations
     *
     * @param position (meters)
     * @return position (rotations)
     */
    static double convertMetersToRotations(double position) {
        return Units.radiansToRotations(convertMetersToRadians(position));
    }

    /**
     * Utility method to convert meters to radians
     *
     * @param position (meters)
     * @return position (radians)
     */
    static double convertMetersToRadians(double position) {
        return position / (Elevator.SPROCKET_RADIUS_METERS * 2);
    }

    public Elevator(ElevatorIO io, BooleanSupplier elevatorBlocked) {
        this.inputs = new ElevatorIOInputsAutoLogged();
        this.io = io;
        this.elevatorBlocked = elevatorBlocked;

        feedForward = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
        motionProfile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                trapezoidMaxVelocity.get(), trapezoidMaxAcceleration.get()));
        currentState = new TrapezoidProfile.State(bottomPositionMeters, 0.0);
        goalState = new TrapezoidProfile.State(bottomPositionMeters, 0.0);
    }

    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        Logger.recordOutput("Elevator/CurrentProfilePosition", currentState.position);
        Logger.recordOutput("Elevator/CurrentProfileVelocity", currentState.velocity);

        Logger.recordOutput("Elevator/Zeroed", zeroed);

        double feedForwardVoltage = 0;
        if (!zeroed) {
            if (DriverStation.isEnabled()
                    && zeroedDebouncer.calculate(
                            MathUtil.isNear(0.0, inputs.velocityMetersPerSecond, 0.05))) {
                io.setElevatorVoltage(0.0);
                // io.resetPosition();
                zeroed = true;
            } else {
                io.setElevatorVoltage(-zeroingVoltage.get());
            }
        } else if (doProfiling) {
            var newState = motionProfile.calculate(0.02, currentState, goalState);
            feedForwardVoltage =
                    feedForward.calculateWithVelocities(currentState.velocity, newState.velocity);
            io.setElevatorState(currentState.position, currentState.velocity, feedForwardVoltage);
            currentState = newState;
        }
        Logger.recordOutput("Elevator/FeedForwardVoltage", feedForwardVoltage);

        boolean checkPosition =
                MathUtil.isNear(
                        convertMetersToRadians(goalState.position),
                        convertMetersToRadians(inputs.positionMeters),
                        positionToleranceMeters);
        boolean checkVelocity =
                MathUtil.isNear(
                        convertMetersToRadians(goalState.velocity),
                        convertMetersToRadians(inputs.velocityMetersPerSecond),
                        velocityToleranceMeters);
        atSetpoint = checkPosition && checkVelocity;
        Logger.recordOutput("Elevator/AtSetpoint", atSetpoint);

        if (kP.hasChanged(this.hashCode()) || kD.hasChanged(this.hashCode())) {
            io.updatePID(kP.getAsDouble(), kD.getAsDouble());
        }

        if (kS.hasChanged(this.hashCode())
                || kV.hasChanged(this.hashCode())
                || kA.hasChanged(this.hashCode())
                || kG.hasChanged(this.hashCode())) {
            feedForward.setKs(kS.get());
            feedForward.setKv(kV.get());
            feedForward.setKa(kA.get());
            feedForward.setKg(kG.get());
        }

        if (trapezoidMaxVelocity.hasChanged(this.hashCode())
                || trapezoidMaxAcceleration.hasChanged(this.hashCode())) {
            motionProfile =
                    new TrapezoidProfile(
                            new TrapezoidProfile.Constraints(
                                    trapezoidMaxVelocity.get(), trapezoidMaxAcceleration.get()));
        }

        coralBlockingAlert.set(this.elevatorBlocked.getAsBoolean());
        Logger.recordOutput("PeriodicTime/Elevator", (Timer.getFPGATimestamp() - startTime) * 1000);
    }

    public void updateProfileStates() {
        currentState.position = inputs.positionMeters;
        currentState.velocity = inputs.velocityMetersPerSecond;
    }

    public Command openLoopElevatorUp() {
        return this.startEnd(
                () -> {
                    this.doProfiling = false;
                    io.setElevatorVoltage(openLoopVoltage.getAsDouble());
                },
                () -> io.setElevatorVoltage(0.0));
    }

    public Command openLoopElevatorDown() {
        return this.startEnd(
                () -> {
                    this.doProfiling = false;
                    io.setElevatorVoltage(-openLoopVoltage.getAsDouble());
                },
                () -> io.setElevatorVoltage(0.0));
    }

    private Command closedLoopToPosition(DoubleSupplier heightSupplier) {
        return this.runOnce(
                        () -> {
                            if (!this.elevatorBlocked.getAsBoolean()) {
                                goalState.position = heightSupplier.getAsDouble();
                                atSetpoint = false;
                                doProfiling = true;
                            }
                        })
                .andThen(Commands.waitUntil(() -> atSetpoint));
    }

    public Command goToStationHeight() {
        return closedLoopToPosition(() -> stationIntakeHeight.get());
    }

    public Command elevatorUp(ReefLevel reefLevel) {
        return closedLoopToPosition(
                () -> reefLevel.getHeightInMeters() - elevatorOffsetMeters.get());
    }

    public Command elevatorUpAlgae(ReefLevel algaeLevel) {
        return closedLoopToPosition(
                () ->
                        algaeLevel.getHeightInMeters()
                                - elevatorOffsetMeters.get()
                                - algaeOffsetMeters.get());
    }

    public Command bumpAlgae(ReefLevel algaeLevel) {
        return closedLoopToPosition(
                () ->
                        algaeLevel.getHeightInMeters()
                                - elevatorOffsetMeters.get()
                                - algaeBumpOffsetMeters.get());
    }

    public Command elevatorDown() {
        return closedLoopToPosition(() -> bottomPositionMeters);
    }

    public boolean isAtBottom() {
        return goalState.position == bottomPositionMeters && atSetpoint;
    }

    public boolean isAtL4() {
        return goalState.position == ReefLevel.L4.getHeightInMeters() - elevatorOffsetMeters.get()
                && atSetpoint;
    }
}
