package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import frc.robot.util.PIDController;

public class IntakeIOSim implements IntakeIO {
    private static final double ARM_LENGTH_METERS = 0.560528;
    private static final double ARM_MASS_KG = 6.57;

    private final DCMotorSim leftMotorSim;
    private final DCMotorSim rightMotorSim;
    private final SingleJointedArmSim pivotSim;

    private final PIDController pivotController;

    private double requestedLeftVoltage = 0;
    private double requestedRightVoltage = 0;
    private double requestedPivotVoltage = 0;
    private boolean openLoop = true;
    private double closedLoopPosition = 0;
    private double closedLoopVelocity = 0;
    private double feedforwardVoltage = 0;

    public IntakeIOSim() {
        DCMotor rollerMotor = DCMotor.getNEO(1);
        leftMotorSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(rollerMotor, 0.001, 1), rollerMotor);
        rightMotorSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(rollerMotor, 0.001, 1), rollerMotor);
        pivotSim =
                new SingleJointedArmSim(
                        DCMotor.getKrakenX60(1),
                        Intake.PIVOT_GEAR_RATIO,
                        SingleJointedArmSim.estimateMOI(ARM_LENGTH_METERS, ARM_MASS_KG),
                        ARM_LENGTH_METERS, // center of mass to pivot point times 2
                        Intake.PIVOT_MIN_ANGLE.in(Radians),
                        Intake.PIVOT_MAX_ANGLE.in(Radians),
                        false,
                        Intake.PIVOT_STARTING_ANGLE.in(Radians));
        pivotController = new PIDController(Intake.PIVOT_kP.get(), 0, Intake.PIVOT_kD.get());
        pivotController.enableContinuousInput(-0.5, 0.5);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        leftMotorSim.setInputVoltage(MathUtil.clamp(requestedLeftVoltage, -12, 12));
        rightMotorSim.setInputVoltage(MathUtil.clamp(requestedRightVoltage, -12, 12));
        leftMotorSim.update(0.02);
        rightMotorSim.update(0.02);

        inputs.leftConnected = true;
        inputs.leftMotorVelocityRadPerSec = leftMotorSim.getAngularVelocity().in(RadiansPerSecond);
        inputs.leftMotorAppliedVolts = requestedLeftVoltage;
        inputs.leftMotorAmps = leftMotorSim.getCurrentDrawAmps();

        inputs.rightConnected = true;
        inputs.rightMotorVelocityRadPerSec =
                rightMotorSim.getAngularVelocity().in(RadiansPerSecond);
        inputs.rightMotorAppliedVolts = requestedRightVoltage;
        inputs.rightMotorAmps = rightMotorSim.getCurrentDrawAmps();

        double appliedVolts;
        if (openLoop) {
            appliedVolts = requestedPivotVoltage;
        } else {
            appliedVolts =
                    pivotController.calculate(
                                    Units.radiansToRotations(inputs.pivotPositionRadians),
                                    closedLoopPosition,
                                    closedLoopVelocity)
                            + feedforwardVoltage;
        }

        pivotSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
        pivotSim.update(0.02);

        inputs.pivotMotorConnected = true;
        inputs.pivotPositionRadians = pivotSim.getAngleRads();
        inputs.pivotVelocityRadPerSec = pivotSim.getVelocityRadPerSec();
        inputs.pivotMotorVoltageVolts = appliedVolts;
        inputs.pivotMotorCurrentAmps = Math.abs(pivotSim.getCurrentDrawAmps());
    }

    @Override
    public void setRollerVoltage(double leftVoltageVolts, double rightVoltageVolts) {
        requestedLeftVoltage = leftVoltageVolts;
        requestedRightVoltage = rightVoltageVolts;
    }

    @Override
    public void setPivotVoltage(double voltage) {
        requestedPivotVoltage = voltage;
        openLoop = true;
    }

    @Override
    public void setPivotState(
            double targetPosition, double targetVelocity, double feedforwardValue) {
        closedLoopPosition = targetPosition;
        closedLoopVelocity = targetVelocity;
        feedforwardVoltage = feedforwardValue;
        openLoop = false;
    }

    @Override
    public void setPivotPosition(double targetPosition) {
        closedLoopPosition = targetPosition;
        closedLoopVelocity = 0;
        feedforwardVoltage = 0;
        openLoop = false;
    }

    @Override
    public void updatePID(double kP, double kD) {
        pivotController.setP(kP);
        pivotController.setD(kD);
    }
}
