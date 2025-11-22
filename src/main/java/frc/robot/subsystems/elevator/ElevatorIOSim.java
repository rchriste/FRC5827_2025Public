package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import frc.robot.util.PIDController;

public class ElevatorIOSim implements ElevatorIO {

    private final ElevatorSim elevatorSim;
    private final DCMotor elevatorMotors = DCMotor.getKrakenX60(2);
    private final PIDController controller;

    private double requestedVoltage = 0.0;
    private double closedLoopPosition = 0.0;
    private double closedLoopVelocity = 0.0;
    private double feedForwardVoltage = 0.0;
    private boolean openLoop = false;

    public ElevatorIOSim() {
        elevatorSim =
                new ElevatorSim(
                        elevatorMotors,
                        Elevator.GEAR_REDUCTION,
                        Pounds.of(13).in(Kilogram),
                        Elevator.SPROCKET_RADIUS_METERS,
                        0.0,
                        Units.feetToMeters(60),
                        true,
                        0);

        controller = new PIDController(Elevator.kP.getAsDouble(), 0, Elevator.kD.getAsDouble());
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (openLoop) {
            elevatorSim.setInputVoltage(MathUtil.clamp(requestedVoltage, -12.0, 12.0));
        } else {
            elevatorSim.setInputVoltage(
                    MathUtil.clamp(
                            controller.calculate(
                                            Elevator.convertMetersToRotations(
                                                    inputs.positionMeters),
                                            Elevator.convertMetersToRotations(closedLoopPosition),
                                            Elevator.convertMetersToRotations(closedLoopVelocity))
                                    + feedForwardVoltage,
                            -12.0,
                            12.0));
        }

        elevatorSim.update(0.02);

        inputs.elevatorConnected = true;
        inputs.positionMeters = elevatorSim.getPositionMeters();
        inputs.velocityMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();
        inputs.leadMotorAppliedVolts = inputs.followMotorAppliedVolts = elevatorSim.getInput(0);
        // There is two motors so divide the current by two
        inputs.followMotorCurrentAmps =
                inputs.leadMotorCurrentAmps = elevatorSim.getCurrentDrawAmps() / 2;
    }

    @Override
    public void setElevatorVoltage(double voltage) {
        requestedVoltage = voltage;
        openLoop = true;
    }

    @Override
    public void setElevatorState(
            double targetPositionMeters, double targetVelocityMpS, double feedForwardVoltage) {
        closedLoopPosition = targetPositionMeters;
        closedLoopVelocity = targetVelocityMpS;
        this.feedForwardVoltage = feedForwardVoltage;
        openLoop = false;
    }

    @Override
    public void setElevatorPosition(double targetPositionMeters) {
        closedLoopPosition = targetPositionMeters;
        closedLoopVelocity = 0;
        feedForwardVoltage = 0;
        openLoop = false;
    }

    public void resetPosition() {
        elevatorSim.setState(0, 0);
    }

    @Override
    public void updatePID(double kP, double kD) {
        controller.setP(kP);
        controller.setD(kD);
    }
}
