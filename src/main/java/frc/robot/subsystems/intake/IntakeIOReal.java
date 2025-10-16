package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SparkUtil;

import java.util.function.DoubleSupplier;

public class IntakeIOReal implements IntakeIO {
    private static final int CURRENT_LIMIT = 40;

    private static final int INTAKE_CORAL_SENSOR = 2;

    private final SparkMax leftMotor, rightMotor;
    private final RelativeEncoder leftEncoder, rightEncoder;
    private final TalonFX pivotMotor;
    private final TalonFXConfiguration pivotConfig;
    private final DigitalInput coralSensor;

    private final PositionVoltage positionRequest;

    private final StatusSignal<Double> pivotTargetSignal;
    private final StatusSignal<Double> pivotErrorSignal;
    private final StatusSignal<Angle> pivotPositionSignal;
    private final StatusSignal<AngularVelocity> pivotVelocitySignal;
    private final StatusSignal<Voltage> pivotMotorVoltageSignal;
    private final StatusSignal<Current> pivotMotorCurrentSignal;
    private final StatusSignal<Temperature> pivotMotorTempSignal;

    // Connection debouncers
    private final Debouncer leftRollerDebouncer = new Debouncer(0.5);
    private final Debouncer rightRollerDebouncer = new Debouncer(0.5);

    public IntakeIOReal() {
        leftMotor = new SparkMax(Constants.intakeLeftMotorCanbus_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.intakeRightMotorCanbus_ID, MotorType.kBrushless);

        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        // brake prevents overshoot when coasting
        rollerConfig
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12)
                .smartCurrentLimit(CURRENT_LIMIT);
        rollerConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
        rollerConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs(20)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        SparkUtil.tryUntilOk(
                rightMotor,
                5,
                () ->
                        rightMotor.configure(
                                rollerConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters));
        rollerConfig.inverted(true);
        SparkUtil.tryUntilOk(
                leftMotor,
                5,
                () ->
                        leftMotor.configure(
                                rollerConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters));

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        coralSensor = new DigitalInput(INTAKE_CORAL_SENSOR);

        pivotMotor = new TalonFX(20);
        // default to up
        positionRequest = new PositionVoltage(Intake.PIVOT_STARTING_ANGLE);
        pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.ClosedLoopGeneral.ContinuousWrap = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.Feedback.SensorToMechanismRatio = Intake.PIVOT_GEAR_RATIO;
        pivotConfig.Voltage.PeakForwardVoltage = 10.0;
        pivotConfig.Voltage.PeakReverseVoltage = -10.0;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                Intake.PIVOT_MAX_ANGLE.in(Rotations);
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                Intake.PIVOT_MIN_ANGLE.in(Rotations);

        pivotConfig.Slot0.kP = Intake.PIVOT_kP.get();
        pivotConfig.Slot0.kD = Intake.PIVOT_kD.get();
        PhoenixUtil.tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> pivotMotor.setPosition(Intake.PIVOT_STARTING_ANGLE));
        pivotMotor.setVoltage(0);

        pivotTargetSignal = pivotMotor.getClosedLoopReference();
        pivotErrorSignal = pivotMotor.getClosedLoopError();
        pivotPositionSignal = pivotMotor.getPosition();
        pivotVelocitySignal = pivotMotor.getVelocity();
        pivotMotorVoltageSignal = pivotMotor.getMotorVoltage();
        pivotMotorCurrentSignal = pivotMotor.getSupplyCurrent();
        pivotMotorTempSignal = pivotMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                pivotTargetSignal,
                pivotErrorSignal,
                pivotPositionSignal,
                pivotVelocitySignal,
                pivotMotorVoltageSignal,
                pivotMotorCurrentSignal,
                pivotMotorTempSignal);
        ParentDevice.optimizeBusUtilizationForAll(pivotMotor);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.hasCoral = getCoralStatus();

        SparkUtil.sparkStickyFault = false;
        SparkUtil.ifOk(
                leftMotor,
                leftEncoder::getVelocity,
                (value) ->
                        inputs.leftMotorVelocityRadPerSec =
                                Units.rotationsPerMinuteToRadiansPerSecond(value));
        SparkUtil.ifOk(
                leftMotor,
                new DoubleSupplier[] {leftMotor::getAppliedOutput, leftMotor::getBusVoltage},
                (values) -> inputs.leftMotorAppliedVolts = values[0] * values[1]);
        SparkUtil.ifOk(
                leftMotor, leftMotor::getOutputCurrent, (value) -> inputs.leftMotorAmps = value);
        SparkUtil.ifOk(
                leftMotor,
                leftMotor::getMotorTemperature,
                (value) -> inputs.leftMotorTempCelsius = value);
        inputs.leftConnected = leftRollerDebouncer.calculate(!SparkUtil.sparkStickyFault);

        SparkUtil.sparkStickyFault = false;
        SparkUtil.ifOk(
                rightMotor,
                rightEncoder::getVelocity,
                (value) ->
                        inputs.rightMotorVelocityRadPerSec =
                                Units.rotationsPerMinuteToRadiansPerSecond(value));
        SparkUtil.ifOk(
                rightMotor,
                new DoubleSupplier[] {rightMotor::getAppliedOutput, rightMotor::getBusVoltage},
                (values) -> inputs.rightMotorAppliedVolts = values[0] * values[1]);
        SparkUtil.ifOk(
                rightMotor, rightMotor::getOutputCurrent, (value) -> inputs.rightMotorAmps = value);
        SparkUtil.ifOk(
                rightMotor,
                rightMotor::getMotorTemperature,
                (value) -> inputs.rightMotorTempCelsius = value);
        inputs.rightConnected = rightRollerDebouncer.calculate(!SparkUtil.sparkStickyFault);

        BaseStatusSignal.refreshAll(
                pivotTargetSignal,
                pivotErrorSignal,
                pivotPositionSignal,
                pivotVelocitySignal,
                pivotMotorVoltageSignal,
                pivotMotorCurrentSignal,
                pivotMotorTempSignal);

        inputs.pivotMotorConnected = pivotMotor.isConnected();
        inputs.pivotPositionRadians =
                Units.rotationsToRadians(pivotPositionSignal.getValueAsDouble());
        inputs.pivotVelocityRadPerSec =
                Units.rotationsToRadians(pivotVelocitySignal.getValueAsDouble());
        inputs.pivotMotorVoltageVolts = pivotMotorVoltageSignal.getValueAsDouble();
        inputs.pivotMotorCurrentAmps = pivotMotorCurrentSignal.getValueAsDouble();
        inputs.pivotMotorTempCelsius = pivotMotorTempSignal.getValueAsDouble();

        inputs.hasCoral = !coralSensor.get();
    }

    // to be implemented when sensor is implemented
    private boolean getCoralStatus() {
        return true;
    }

    @Override
    public void setRollerVoltage(double leftVoltageVolts, double rightVoltageVolts) {
        leftMotor.setVoltage(leftVoltageVolts);
        rightMotor.setVoltage(rightVoltageVolts);
    }

    @Override
    public void setPivotBrakeMode(boolean brake) {
        pivotMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
    }

    @Override
    public void setPivotState(
            double targetPosition, double targetVelocity, double feedforwardValue) {
        pivotMotor.setControl(
                positionRequest
                        .withPosition(targetPosition)
                        .withVelocity(targetVelocity)
                        .withFeedForward(feedforwardValue));
    }

    @Override
    public void setPivotPosition(double targetPosition) {
        pivotMotor.setControl(
                positionRequest.withPosition(targetPosition).withVelocity(0).withFeedForward(0));
    }

    @Override
    public void updatePID(double kP, double kD) {
        pivotConfig.Slot0.kP = kP;
        pivotConfig.Slot0.kD = kD;
        PhoenixUtil.tryUntilOk(
                5, () -> pivotMotor.getConfigurator().apply(pivotConfig.Slot0, 0.25));
    }
}
