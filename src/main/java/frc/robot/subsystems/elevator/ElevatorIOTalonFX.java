package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {

    private static final double CURRENT_LIMIT = 40.0;

    private final TalonFX leadMotor, followMotor;
    private final TalonFXConfiguration config;
    private final PositionVoltage positionRequest;

    public StatusSignal<Angle> position;
    public StatusSignal<AngularVelocity> velocity;

    public StatusSignal<Voltage> leadAppliedVolts;
    public StatusSignal<Voltage> followAppliedVolts;

    public StatusSignal<Current> leadCurrentAmps;
    public StatusSignal<Current> followCurrentAmps;

    public StatusSignal<Temperature> followMotorTemperature;
    public StatusSignal<Temperature> leadMotorTemperature;

    private final VoltageOut leadVoltageOut = new VoltageOut(0.0);

    private double convertRotationsToMeters(double rotations) {
        return Elevator.SPROCKET_RADIUS_METERS
                * Units.rotationsToRadians(rotations)
                * 2; // Multiply by 2 because elevator is cascading
    }

    public ElevatorIOTalonFX() {

        positionRequest = new PositionVoltage(Elevator.bottomPositionMeters);
        this.leadMotor = new TalonFX(Constants.elevatorLeadMotorOpenCanbus_ID);
        this.followMotor = new TalonFX(Constants.elevatorFollowMotorOpenCanbus_ID);
        followMotor.setControl(
                new Follower(
                        leadMotor.getDeviceID(),
                        false)); // Follower keeps both motors synced; false means same direction.
        config = new TalonFXConfiguration();

        // Configure motor/controller behavior. These numbers should match the real mechanism.
        config.Feedback.SensorToMechanismRatio = Elevator.GEAR_REDUCTION;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Safety limits:
        // Current limit protects motors and wiring if elevator stalls.
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;

        // Soft limits prevent driving past physical travel and damaging the robot.
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                Elevator.convertMetersToRotations(1.7);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                Elevator.convertMetersToRotations(-0.01);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // Slot0 PID values for position control (kept in sync with Elevator.kP/kD tunables).
        config.Slot0.kP = Elevator.kP.getAsDouble();
        config.Slot0.kD = Elevator.kD.getAsDouble();

        PhoenixUtil.tryUntilOk(5, () -> leadMotor.getConfigurator().apply(config, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> followMotor.getConfigurator().apply(config, 0.25));

        this.position = leadMotor.getPosition();
        this.velocity = leadMotor.getVelocity();
        this.leadAppliedVolts = leadMotor.getMotorVoltage();
        this.followAppliedVolts = followMotor.getMotorVoltage();
        this.leadCurrentAmps = leadMotor.getSupplyCurrent();
        this.followCurrentAmps = followMotor.getSupplyCurrent();
        this.leadMotorTemperature = leadMotor.getDeviceTemp();
        this.followMotorTemperature = followMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                this.position,
                this.velocity,
                this.leadAppliedVolts,
                this.followAppliedVolts,
                this.leadCurrentAmps,
                this.followCurrentAmps,
                this.leadMotorTemperature,
                this.followMotorTemperature);

        ParentDevice.optimizeBusUtilizationForAll(leadMotor);
        ParentDevice.optimizeBusUtilizationForAll(followMotor);

        // reset motor positions
        leadMotor.setPosition(0);
        followMotor.setPosition(0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        var elevatorStatus =
                BaseStatusSignal.refreshAll(
                        this.position,
                        this.velocity,
                        this.leadAppliedVolts,
                        this.followAppliedVolts,
                        this.leadCurrentAmps,
                        this.followCurrentAmps,
                        this.leadMotorTemperature,
                        this.followMotorTemperature);
        inputs.elevatorConnected = elevatorStatus.isOK();
        inputs.positionMeters = convertRotationsToMeters(position.getValueAsDouble());
        inputs.velocityMetersPerSecond = convertRotationsToMeters(velocity.getValueAsDouble());
        inputs.leadMotorAppliedVolts = leadAppliedVolts.getValueAsDouble();
        inputs.followMotorAppliedVolts = followAppliedVolts.getValueAsDouble();
        inputs.leadMotorCurrentAmps = leadCurrentAmps.getValueAsDouble();
        inputs.followMotorCurrentAmps = followCurrentAmps.getValueAsDouble();
        inputs.leadMotorTemperature = leadMotorTemperature.getValueAsDouble();
        inputs.followMotorTemperature = followMotorTemperature.getValueAsDouble();
    }

    @Override
    public void setElevatorVoltage(double voltage) {
        leadMotor.setControl(leadVoltageOut.withOutput(voltage));
    }

    @Override
    public void setElevatorPosition(double targetPositionMeters) {
        leadMotor.setControl(
                positionRequest.withPosition(
                        Elevator.convertMetersToRotations(targetPositionMeters)));
    }

    @Override
    public void setElevatorState(
            double targetPositionMeters, double targetVelocityMpS, double feedForwardVoltage) {
        // Position + velocity + feedforward control.
        // FeedforwardVoltage here is the "predictive" part that helps the elevator move quickly and
        // smoothly under load.
        leadMotor.setControl(
                positionRequest
                        .withPosition(Elevator.convertMetersToRotations(targetPositionMeters))
                        .withVelocity(Elevator.convertMetersToRotations(targetVelocityMpS))
                        .withFeedForward(feedForwardVoltage));
    }

    public void resetPosition() {
        leadMotor.setPosition(0);
        followMotor.setPosition(0);
    }

    @Override
    public void updatePID(double kP, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kD = kD;
        PhoenixUtil.tryUntilOk(5, () -> leadMotor.getConfigurator().apply(config, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> followMotor.getConfigurator().apply(config, 0.25));
    }
}
